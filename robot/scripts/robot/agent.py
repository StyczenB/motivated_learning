#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.srv import GetModelState, SetModelState
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


class State:
    IDLE = 0
    MOVING = 1
    CHARGING = 2


class Agent:
    BATTERY_MAX = 1
    BATTERY_MIN = 0
    MOVING_DISCHARGE = 0.995

    def __init__(self):
        # self.charging_coeff = 0.001
        self._idle_discharge_coeff = 0.9995
        self._moving_discharge_coeff = Agent.MOVING_DISCHARGE
        # self.wheel_lubrication_moving_discharge_coeff = 0.95
        self._wheel_lubrication_effect_start_time = time.time()

        self._state = State.IDLE
        self._battery_level = Agent.BATTERY_MAX
        self._prev_x, self._prev_y, self._prev_yaw = self.get_robot_pose()
        self._state_pub = rospy.Publisher('agent_state', StateMsg, queue_size=1)

        rospy.loginfo('Waiting for global_world_map message...')
        self._global_world_map = rospy.wait_for_message('global_world_map', MapMsg)
        rospy.loginfo('...obtain global world map')
        self._chargers_manager = ChargersManager()

    def step(self):
        if self._battery_level < 0.1:
            rospy.logwarn('There is less then 0.1 of agent\'s battery...')

        moving = self.is_moving()
        self._state = State.MOVING if moving else State.IDLE
        
        if time.time() - self._wheel_lubrication_effect_start_time > 10:
            # effect of wheel lubrication has just wore off, using default moving discharge value
            self._moving_discharge_coeff = Agent.MOVING_DISCHARGE

        curr_field = self.get_current_field()
        # Agent stopped on this field, not only going through it
        if not moving:
            if curr_field.type == Field.WHEEL_LUBRICATION:
                self._moving_discharge_coeff = 1.0
                self._wheel_lubrication_effect_start_time = time.time()  # effect is valid for 10 seconds
                rospy.loginfo('Stepped on WHEEL_LUBRICATION field. Agent for 10 seconds will not use battery charge.')
            elif curr_field.type == Field.CHARGER and self._battery_level < Agent.BATTERY_MAX:
                self._state = State.CHARGING
            else:
                self._state = State.IDLE

        if self._state == State.IDLE:
            rospy.loginfo('State.IDLE')
            self._battery_level *= self._idle_discharge_coeff
        elif self._state == State.MOVING:
            rospy.loginfo('State.MOVING')
            self._battery_level *= self._moving_discharge_coeff
        elif self._state == State.CHARGING:
            rospy.loginfo('State.CHARGING')
            if self._battery_level < Agent.BATTERY_MAX:
                charger = self._chargers_manager.get_charger(x=curr_field.coords.x, y=curr_field.coords.y)
                if charger:
                    energy = charger.draw_energy()
                    self._battery_level += energy
                else:
                    rospy.logwarn(f'Problem getting charger from coordinates: x={curr_field.coords.x}, y={curr_field.coords.y}')
            else:
                rospy.loginfo('Agent\'s battery is full')
        else:
            rospy.logerr('Invalid agent state.')
            rospy.signal_shutdown('Invalid agent state. Exiting...')

        # clamp battery level
        self._battery_level = min(max(Agent.BATTERY_MIN, self._battery_level), Agent.BATTERY_MAX)

        agent_state = self.publish()
        self._prev_x, self._prev_y, self._prev_yaw = agent_state.coords.x, agent_state.coords.y, agent_state.yaw
        self._chargers_manager.update()

    def get_field_from_global_map(self, x: int, y: int):
        for field in self._global_world_map.fields:
            if x == field.coords.x and y == field.coords.y:
                return field
        rospy.logwarn(f'Could not find valid field with coordinates: x: {x}, y: {y}')
        return None

    def is_moving(self) -> bool:
        x_pos, y_pos, yaw = self.get_robot_pose()
        x_changed = abs(x_pos - self._prev_x) > 0.001
        y_changed = abs(y_pos - self._prev_y) > 0.001
        yaw_changed = abs(yaw - self._prev_yaw) > 0.01
        return x_changed or y_changed or yaw_changed

    def get_robot_pose(self) -> (float, float, float):
        odom = rospy.wait_for_message('/odom', Odometry)
        pos = odom.pose.pose.position.x, odom.pose.pose.position.y
        quat = odom.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return pos[0], pos[1], yaw
    
    def get_current_field(self):
        curr_grid_coord = rospy.wait_for_message('grid_coord', Point)
        curr_field = self.get_field_from_global_map(curr_grid_coord.x, curr_grid_coord.y)
        return curr_field

    def publish(self) -> StateMsg:
        agent_state = StateMsg()
        agent_state.header.stamp = rospy.Time().now()
        agent_state.battery_level = self._battery_level
        agent_state.state = self._state
        agent_state.coords.x, agent_state.coords.y, agent_state.yaw = self.get_robot_pose() 
        self._state_pub.publish(agent_state)
        return agent_state


def check_simulation_state():
    # if this function returns gazebo is running
    link_states = rospy.wait_for_message('/gazebo/link_states', LinkStates)


if __name__ == '__main__':
    try:
        print('start')
        rospy.init_node('robot', anonymous=True)
        agent = Agent()
        rospy.loginfo('Started node with agent state updaters.')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            check_simulation_state()
            agent.step()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
