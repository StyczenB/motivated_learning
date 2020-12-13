#!/usr/bin/env python3
from typing import List
import rospy
from robot_msgs.msg import AgentStateMsg, MapMsg, FieldMsg
from environment.chargers_manager import ChargersManagerClient
# from environment.global_map_manager import Field
from environment.gazebo_client import GazeboClient
from robot.robot_pose_manager import RobotPoseManagerClient  # RobotPoseManager
from robot.battery_manager import BatteryManager


class State:
    IDLE = AgentStateMsg.IDLE
    MOVING = AgentStateMsg.MOVING
    CHARGING = AgentStateMsg.CHARGING
    state_names = ['IDLE', 'MOVING', 'CHARGING']


class Agent:
    def __init__(self):
        self._wheel_lubrication_effect_start_time = GazeboClient.get_sim_time()
        self._last_home_visit = GazeboClient.get_sim_time()
        self._internal_map = MapMsg()
        self._state = State.IDLE
        self._state_pub = rospy.Publisher('agent_state', AgentStateMsg, queue_size=1, latch=True)

        rospy.loginfo('Waiting for global_world_map message...')
        self._global_world_map: MapMsg = rospy.wait_for_message('global_world_map', MapMsg)
        rospy.loginfo('...obtain global world map')

        self._chargers_mngr_client = ChargersManagerClient()
        self._pos_mngr_client = RobotPoseManagerClient()
        self._battery_mngr = BatteryManager()

    def __del__(self):
        self._state_pub.unregister()

    def step(self):
        rospy.logdebug('Agent.step called')
        
        if self._battery_mngr.level < 0.1:
            rospy.logwarn('There is less then 0.1 of agent\'s battery...')

        current_coords = self._pos_mngr_client.coords
        if current_coords.x == 0 and current_coords.y == 0:
            self._last_home_visit = GazeboClient.get_sim_time()

        moving = self._pos_mngr_client.is_moving()
        self._state = State.MOVING if moving else State.IDLE

        if GazeboClient.get_sim_time() - self._wheel_lubrication_effect_start_time > 10:
            # effect of wheel lubrication has just wore off, using default moving discharge value
            self._battery_mngr.reset_discharge()

        curr_field = self.get_current_field()
        self.update_internal_map(curr_field)

        # Agent stopped on this field, not only going through it
        if not moving:
            if curr_field.type == FieldMsg.WHEEL_LUBRICATION:
                self._battery_mngr.no_draining()
                # effect is valid for 10 seconds
                self._wheel_lubrication_effect_start_time = GazeboClient.get_sim_time()
                # rospy.loginfo('Stepped on WHEEL_LUBRICATION field. Agent for 10 seconds will not use battery charge.')
            elif curr_field.type == FieldMsg.CHARGER and self._battery_mngr.level < BatteryManager.BATTERY_MAX:
                self._state = State.CHARGING
            else:
                self._state = State.IDLE

        if self._state == State.IDLE:
            rospy.logdebug('State.IDLE')
            self._battery_mngr.idle_discharge()
        elif self._state == State.MOVING:
            rospy.logdebug('State.MOVING')
            self._battery_mngr.moving_discharge()
        elif self._state == State.CHARGING:
            rospy.logdebug('State.CHARGING')
            if self._battery_mngr.level < BatteryManager.BATTERY_MAX:
                energy = self._chargers_mngr_client.get_energy_from_charger(x=curr_field.coords.x, y=curr_field.coords.y)
                self._battery_mngr.update_battery(energy)
            else:
                rospy.logdebug('Agent\'s battery is full')
        else:
            rospy.logerr('Invalid agent state.')
            rospy.signal_shutdown('Invalid agent state. Exiting...')
        self.publish()

    def get_current_field(self) -> FieldMsg:
        curr_field = self.get_field_from_global_map(self._pos_mngr_client.coords.x, self._pos_mngr_client.coords.y)
        return curr_field

    def get_field_from_global_map(self, x: int, y: int) -> FieldMsg:
        for field in self._global_world_map.fields:
            if x == field.coords.x and y == field.coords.y:
                return field
        rospy.logwarn(f'Could not find valid field with coordinates: x: {x}, y: {y}')
        return FieldMsg(name='invalid_field', nr_visits=-1, type=-1)

    def update_internal_map(self, current_field: FieldMsg):
        for field in self._internal_map.fields:
            if field.coords.x == current_field.coords.x and field.coords.y == current_field.coords.y:
                field.nr_visits += 1
                break
        else:
            current_field.nr_visits += 1
            self._internal_map.fields.append(current_field)

    def publish(self) -> AgentStateMsg:
        agent_state = AgentStateMsg()
        agent_state.header.stamp = rospy.Time.from_sec(GazeboClient.get_sim_time())
        agent_state.coords = self._pos_mngr_client.coords
        agent_state.pose = self._pos_mngr_client.pose
        agent_state.battery_level = self._battery_mngr.level
        agent_state.state = self._state
        agent_state.state_str = State.state_names[self._state]
        agent_state.last_home_visit = self._last_home_visit
        agent_state.wheel_lubrication_counter = 0
        agent_state.internal_map = self._internal_map
        self._state_pub.publish(agent_state)
        return agent_state
