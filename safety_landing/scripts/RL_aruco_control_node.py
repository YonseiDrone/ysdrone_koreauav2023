#!/usr/bin/env python3
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from math import pow, atan2, sqrt, pi, degrees
from std_msgs.msg import Float32MultiArray, Float32
import onnxruntime, rospkg, onnx
import numpy as np
from koreauav_utils import auto_service, math_utils


class RLControl:
    '''
    Class for precisely landing the UAV using reinforcement learning.
    Training was conducted in the Unity envrionment. And Save the paramters learned in the Unity environment as an onnx file.
    Algorithms: 
        Soft Actor-Critc(SAC)
    '''
    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_vel = TwistStamped()
        self.relative_dis = Float32MultiArray()
        self.landing_velocity_position = PoseStamped()
        self.landing_velocity = PositionTarget()
        self.mission = 0

        #Subscriber
        self.relative_dis_sub = rospy.Subscriber("/relative_distance", Float32MultiArray, self.relative_dis_cb)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.vel_cb)
        self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)

        #Publisher
        # Send the desired velocity of UAV to control_node in offboard package.
        self.landing_vel_position_pub = rospy.Publisher('/landing_velocity_position', PoseStamped, queue_size=1)
        self.landing_vel_pub = rospy.Publisher("/landing_velocity", PositionTarget, queue_size=1)

        self.scale = 0.2
        self.z_offset = 0.0
        self.rospack = rospkg.RosPack()
        self.onnxPath = self.rospack.get_path('safety_landing') + '/scripts/DroneLanding-3855533.onnx'

        self.onnx_model = onnx.load(self.onnxPath)
        onnx.checker.check_model(self.onnx_model)
        self.model = onnxruntime.InferenceSession(self.onnxPath)
        
        self.yaw = 90*math.pi/180 # desired yaw angle.
        
        # Initialization
        self.relative_dis.data = [0, 0, 0]
    
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
        '''
        Since the coordinate system used in the Unity and the coordinated system in mavros(ENU) are different,
        so adjust as below.
        '''
        state = []
        if len(self.relative_dis.data) >= 3:
            state.append(self.current_vel.twist.linear.y)
            state.append(self.current_vel.twist.linear.z)
            state.append(self.current_vel.twist.linear.x)
            state.append(self.relative_dis.data[0])
            state.append(self.current_pose.pose.position.z)
            state.append(self.relative_dis.data[1])
        return state

    def action(self, e):
        '''
        1) The value coming from the aurco_VIO node is nan:
            Position control is performed.
            It means that aruco marker is not recognized.s
        2) The valuse is "not" nan:
            Velocity control is performed through RL control.
            It means that aruco marker is recognized.
        '''
        if self.mission == 10:
            state = self.get_state()
            if np.isnan(self.relative_dis.data[0]):
                qx, qy, qz, qw = math_utils.to_quaternion(self.yaw, 0, 0)
                self.landing_velocity_position.pose.position.x = 0
                self.landing_velocity_position.pose.position.y = 0
                self.landing_velocity_position.pose.position.z = self.current_pose.pose.position.z - 0.4
                self.landing_velocity_position.pose.orientation.x = qx
                self.landing_velocity_position.pose.orientation.y = qy
                self.landing_velocity_position.pose.orientation.z = qz
                self.landing_velocity_position.pose.orientation.w = qw
                self.landing_vel_position_pub.publish(self.landing_velocity_position)

            else:
                ort_inputs = {self.model.get_inputs()[0].name: [state]}
                action = self.model.run(None, ort_inputs)
                action = np.multiply(action[2][0], [1, 1, 0.8])
                action = action * self.scale

                self.landing_velocity.velocity.x = action[1]
                self.landing_velocity.velocity.y = action[0]
                self.landing_velocity.velocity.z = action[2]
                self.landing_velocity.yaw = self.yaw
                self.landing_vel_pub.publish(self.landing_velocity)

            '''
            Function to automatically run a service call. 
            This function is defined in "koreauav_utils" package, and is executed if ros parameter("/srv_mode") is false in the waypoint_server.launch file.
            Mission num 11:
                AutoLanding -> Disarming
            '''           
            if self.current_pose.pose.position.z < 0.5:
                auto_service.call_drone_command(11)


if __name__ == "__main__":
    rospy.init_node("RL_aruco_control_node")
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


