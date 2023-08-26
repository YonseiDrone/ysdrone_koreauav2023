#!/usr/bin/env python3
import rospy
import tf
import math
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from math import pow, atan2, sqrt, pi, degrees
from std_msgs.msg import Float32MultiArray, Float32
import onnxruntime, rospkg, onnx
import numpy as np
from koreauav_utils import auto_service


class RLControl:
    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_vel = TwistStamped()
        self.relative_dis = Float32MultiArray()
        self.landing_velocity = Twist()
        self.mission = Float32()

        #Subscriber
        self.relative_dis_sub = rospy.Subscriber("/relative_distance", Float32MultiArray, self.relative_dis_cb)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.vel_cb)
        self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)

        #Publisher
        # Send the desired velocity of UAV to control_node in offboard package.
        self.landing_vel_pub = rospy.Publisher("/landing_velocity", Twist, queue_size=1)

        # controller frequency in Hz
        self.hz = 20.0
        # Limit the rate in which ROS nodes run
        self.rate = rospy.Rate(self.hz)
        self.dt = (1.0 / self.hz)

        self.scale = 0.4
        self.z_offset = 0.0
        self.rospack = rospkg.RosPack()
        self.onnxPath = self.rospack.get_path('safety_landing') + '/scripts/DroneLanding-8078831.onnx'

        self.onnx_model = onnx.load(self.onnxPath)
        onnx.checker.check_model(self.onnx_model)
        self.model = onnxruntime.InferenceSession(self.onnxPath)
 
    def mission_cb(self, msg):
        self.mission = msg.data

    def relative_dis_cb(self, msg):
        self.relative_dis = msg
        #rospy.loginfo(f"{self.relative_dis.data[0]}, {self.relative_dis.data[1]}, {self.relative_dis.data[2]}")

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def vel_cb(self, msg):
        self.current_vel = msg

    def get_state(self):
        state = []
        state.append(self.current_vel.twist.linear.y)
        state.append(self.current_vel.twist.linear.z)
        state.append(self.current_vel.twist.linear.x)
        state.append(self.current_pose.pose.position.x)
        state.append(self.current_pose.pose.position.z + self.z_offset)
        state.append(self.current_pose.pose.position.y)
        return state

    def action(self, e):
        if self.mission == 9:
            state = self.get_state()
            ort_inputs = {self.model.get_inputs()[0].name: [state]}
            action = self.model.run(None, ort_inputs)
            action = np.multiply(action[2][0], [1, 1, 1])
            action = action * self.scale

            self.landing_velocity.linear.x = action[1]
            self.landing_velocity.linear.y = action[0]
            self.landing_velocity.linear.z = action[2]
            #rospy.loginfo(f"action : {action}")
            self.landing_vel_pub.publish(self.landing_velocity)

            if self.current_pose.pose.position.z < 0.3:
                auto_service.call_drone_command(11)


if __name__ == "__main__":
    rospy.init_node("RL_control_node")
    try:
        RL_control_node_handler = RLControl()

        rate = rospy.Rate(100)
        # wait for FCU connection
        while not rospy.is_shutdown() and not RL_control_node_handler.current_state.connected:
            rate.sleep()
        rospy.loginfo("RL_control_node : FCU connected")

        rospy.Timer(rospy.Duration(0.05), RL_control_node_handler.action)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass


