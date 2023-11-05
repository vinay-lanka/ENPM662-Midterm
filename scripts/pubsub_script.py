#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf_transformations import euler_from_quaternion
import numpy as np
import math

# Define key codes
START_POSE = (0,0)
END_POSE = (10,10)


class RobotControlNode(Node):

    def __init__(self):
        super().__init__('robot_control_node')

        self.reached_goal = False

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.state_estimator_and_publisher)

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10)
        self.subscription = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.imu_callback,
            qos_profile)

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile)

        self.pos_x = START_POSE[0]
        self.pos_y = START_POSE[1]

        self.Kp_angle = 0.4
        self.linear_vel = 1.0
        
    def imu_callback(self, msg):
        orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (self.current_roll, self.current_pitch, self.current_yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw += 1.5707
        # print("{:.2f}".format(self.current_yaw))
    
    def joint_state_callback(self, msg):
        self.velocity= (-msg.velocity[0]+msg.velocity[3])/2

    def state_estimator_and_publisher(self):
        if self.reached_goal != True :
            print("Current yaw - ", self.current_yaw)
            self.pos_x += self.velocity * 0.2032 / 4 * np.cos(self.current_yaw)
            self.pos_y += self.velocity * 0.2032 / 4 * np.sin(self.current_yaw)
            # print("{:.2f}".format(self.pos_x), "{:.2f}".format(self.pos_y))

            reference_angle = np.arctan2(END_POSE[1] - self.pos_y, END_POSE[0] - self.pos_x)
            
            output_steer_angle = self.Kp_angle * -(self.current_yaw-reference_angle)

            print("Current steering angle - ",output_steer_angle)

            joint_positions = Float64MultiArray()
            wheel_velocities = Float64MultiArray()
            
            wheel_velocities.data = [-self.linear_vel,self.linear_vel,-self.linear_vel,self.linear_vel]
            joint_positions.data = [output_steer_angle,output_steer_angle]

            self.joint_position_pub.publish(joint_positions)
            self.wheel_velocities_pub.publish(wheel_velocities)
            
            print("Distance - ", math.dist([self.pos_x,self.pos_y],[END_POSE[0],END_POSE[1]]))

            if math.dist([self.pos_x,self.pos_y],[END_POSE[0],END_POSE[1]]) < 0.1:
                wheel_velocities.data = [0.0,0.0,0.0,0.0]
                joint_positions.data = [0.0,0.0]
                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)
                self.reached_goal = True
        else:
            print("Reached")



def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()