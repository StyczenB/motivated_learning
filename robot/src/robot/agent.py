#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.srv import GetModelState, SetModelState, GetWorldProperties
from gazebo_msgs.msg import ModelState, LinkStates
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import StateMsg, MapMsg, FieldMsg
import tf
import math
import copy
import time
from environment.chargers_manager import ChargersManager
from environment.global_map_manager import GlobalMapManager, Field
from robot.robot_pose_manager import RobotPoseManager
from robot.battery_manager import BatteryManager


class State:
    IDLE = 0
    MOVING = 1
    CHARGING = 2


class Agent:
    def __init__(self):
        self._wheel_lubrication_effect_start_time = self.get_sim_time()
        self._state = State.IDLE
        self._state_pub = rospy.Publisher('agent_state', StateMsg, queue_size=1)

        rospy.loginfo('Waiting for global_world_map message...')
        self._global_world_map = rospy.wait_for_message('global_world_map', MapMsg)
        rospy.loginfo('...obtain global world map')
        
        self._chargers_mngr = ChargersManager()
        self._pos_mngr = RobotPoseManager()
        self._battery_mngr = BatteryManager()

    def get_sim_time(self):
        world_properties_client = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        world_properties_client.wait_for_service()
        properties = world_properties_client()
        return properties.sim_time

    def step(self):
        if self._battery_mngr.level < 0.1:
            rospy.logwarn('There is less then 0.1 of agent\'s battery...')

        moving = self._pos_mngr.is_moving()
        self._state = State.MOVING if moving else State.IDLE
        
        if self.get_sim_time() - self._wheel_lubrication_effect_start_time > 10:
            # effect of wheel lubrication has just wore off, using default moving discharge value
            self._battery_mngr.reset_discharge()

        curr_field = self.get_current_field()
        # Agent stopped on this field, not only going through it
        if not moving:
            if curr_field.type == Field.WHEEL_LUBRICATION:
                self._battery_mngr.no_draining()
                self._wheel_lubrication_effect_start_time = self.get_sim_time()  # effect is valid for 10 seconds
                # rospy.loginfo('Stepped on WHEEL_LUBRICATION field. Agent for 10 seconds will not use battery charge.')
            elif curr_field.type == Field.CHARGER and self._battery_mngr.level < BatteryManager.BATTERY_MAX:
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
                charger = self._chargers_mngr.get_charger(x=curr_field.coords.x, y=curr_field.coords.y)
                if charger:
                    energy = charger.draw_energy()
                    self._battery_mngr.update_battery(energy)
                else:
                    rospy.logwarn(f'Problem getting charger from coordinates: x={curr_field.coords.x}, y={curr_field.coords.y}')
            else:
                rospy.logdebug('Agent\'s battery is full')
        else:
            rospy.logerr('Invalid agent state.')
            rospy.signal_shutdown('Invalid agent state. Exiting...')

        agent_state = self.publish()
        self._pos_mngr.step()
        self._chargers_mngr.step()

    def get_field_from_global_map(self, x: int, y: int):
        for field in self._global_world_map.fields:
            if x == field.coords.x and y == field.coords.y:
                return field
        rospy.logwarn(f'Could not find valid field with coordinates: x: {x}, y: {y}')
        return None
    
    def get_current_field(self):
        curr_field = self.get_field_from_global_map(self._pos_mngr.coords.x, self._pos_mngr.coords.y)
        return curr_field

    def publish(self) -> StateMsg:
        agent_state = StateMsg()
        agent_state.header.stamp = rospy.Time(self.get_sim_time())
        agent_state.battery_level = self._battery_mngr.level
        agent_state.state = self._state
        agent_state.coords = self._pos_mngr.coords
        agent_state.pose = self._pos_mngr.pose
        self._state_pub.publish(agent_state)
        return agent_state
