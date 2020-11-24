#!/usr/bin/env python3
import rospy
import sys
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from robot_msgs.msg import State, Map, Field, GridCoord
import tf
import math
import copy
import time


# class State:
#     IDLE = 0
#     MOVING = 1
#     CHARGING = 2


class Agent:
    BATTERY_MAX = 1
    BATTERY_MIN = 0
    MOVING_DISCHARGE = 0.995

    def __init__(self):
        self.charging_coeff = 0.001
        self.idle_discharge_coeff = 0.9995
        self.moving_discharge_coeff = Agent.MOVING_DISCHARGE
        # self.wheel_lubrication_moving_discharge_coeff = 0.95
        self.wheel_lubrication_effect_start_time = time.time()

        self.state = State.IDLE
        self.battery_level = Agent.BATTERY_MAX
        self.prev_x, self.prev_y, self.prev_yaw = self.get_robot_pose()
        self.state_pub = rospy.Publisher('agent_state', State, queue_size=1)

        rospy.loginfo('Waiting for global_world_map message...')
        self.global_world_map = rospy.wait_for_message('global_world_map', Map)
        rospy.loginfo('...obtain global world map')
        # self.map = Map()

    def step(self):
        moving = self.is_moving()
        self.state = State.MOVING if moving else State.IDLE
        
        if time.time() - self.wheel_lubrication_effect_start_time > 10:
            # effect of wheel lubrication has just wore off, using default moving discharge value
            self.moving_discharge_coeff = Agent.MOVING_DISCHARGE

        # Agent stopped on this field, not only going through it
        if not moving:
            curr_field = self.get_current_field()
            if curr_field.type == Field.CHARGER:
                self.state = State.CHARGING
            elif curr_field.type == Field.WHEEL_LUBRICATION:
                self.moving_discharge_coeff = 1.0
                self.wheel_lubrication_effect_start_time = time.time()  # effect is valid for 10 seconds
                rospy.loginfo('Stepped on WHEEL_LUBRICATION field. Agent for 10 seconds.')

        if self.state == State.IDLE:
            rospy.loginfo('State.IDLE')
            self.battery_level *= self.idle_discharge_coeff
        elif self.state == State.MOVING:
            rospy.loginfo('State.MOVING')
            self.battery_level *= self.moving_discharge_coeff
        elif self.state == State.CHARGING:
            rospy.loginfo('State.CHARGING')
            missing_charge_to_max = Agent.BATTERY_MAX - self.battery_level
            self.battery_level += self.charging_coeff * missing_charge_to_max
        else:
            rospy.logerr('Invalid agent state.')
            rospy.signal_shutdown('Invalid agent state. Exiting...')

        # clamp battery levele
        min(max(Agent.BATTERY_MIN, self.battery_level), Agent.BATTERY_MAX)

        agent_state = State()
        agent_state.header.stamp = rospy.Time().now()
        agent_state.battery_level = self.battery_level
        agent_state.state = self.state
        agent_state.x, agent_state.y, agent_state.yaw = self.get_robot_pose() 
        self.state_pub.publish(agent_state)

        self.prev_x, self.prev_y, self.prev_yaw = agent_state.x, agent_state.y, agent_state.yaw


    def get_field_from_global_map(self, x: int, y: int):
        for field in self.global_world_map.cells:
            if x == field.x and y == field.y:
                return field
        rospy.logwarn(f'Could not find valid field with coordinates: x: {x}, y: {y}')
        return None


    def is_moving(self):
        x_pos, y_pos, yaw = self.get_robot_pose()
        x_changed = not math.isclose(x_pos, self.prev_x, rel_tol=1e-3) 
        y_changed = not math.isclose(y_pos, self.prev_y, rel_tol=1e-3) 
        yaw_changed = not math.isclose(yaw, self.prev_yaw, rel_tol=1e-3) 
        return x_changed or y_changed or yaw_changed


    def get_robot_pose(self):
        odom = rospy.wait_for_message('/odom', Odometry)
        pos = odom.pose.pose.position.x, odom.pose.pose.position.y
        quat = odom.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return pos[0], pos[1], yaw

    
    def get_current_field(self):
        curr_grid_coord = rospy.wait_for_message('grid_coord', GridCoord)
        curr_field = self.get_field_from_global_map(curr_grid_coord.x, curr_grid_coord.y)
        return curr_field


if __name__ == '__main__':
    try:
        print('start')
        rospy.init_node('robot', anonymous=True)
        agent = Agent()
        rospy.loginfo('Started node with agent state updaters.')
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            agent.step()

            rate.sleep()
    except rospy.ROSInterruptException:
        pass


