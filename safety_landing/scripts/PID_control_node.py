#!/usr/bin/env python3
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from math import pow, atan2, sqrt, pi, degrees
from std_msgs.msg import Float32MultiArray, Float32
from koreauav_utils import auto_service, math_utils

class PID:
    '''
    Class for PID control
    '''
    def __init__(self, kp=1, kd=0, ki=0, dt=0.01):

        # Gains
        self.kp = kp
        self.kd = kd
        self.ki = ki

        # time step
        self.dt = dt

        # Default Error Initialization
        self.err_previous = 0.00001
        self.err_accmulation = 0

    def compute(self, err):

        # compute dervivative
        err_deriv = (err - self.err_previous) / self.dt

        # update integration
        self.err_accmulation = self.err_accmulation + self.dt * (err + self.err_previous)/2

        # compute pid equation
        pid = self.kp * err + self.kd * err_deriv + self.ki * self.err_accmulation

        # update error
        self.err_previous = err
        return pid

class PIDControl:
    '''
    Class to precisely land the UAV on the aruco marker using PID control.
    Velocity control based distance calculated in "aruco_VIO" node.
    Only x and y are controlled through PID control, and the spped in the z direction was limited to a constant speed.
    '''
    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.relative_dis = Float32MultiArray()
        self.desired_landing_position = PoseStamped()
        self.desired_landing = PositionTarget()
        self.mission = 0

        #Subscriber
        self.relative_dis_sub = rospy.Subscriber("/relative_distance", Float32MultiArray, self.relative_dis_cb)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)

        #Publisher
        # Send the desired velocity of UAV to control_node in offboard package.
        self.desired_landing_position_pub = rospy.Publisher('/desired_landing_position', PoseStamped, queue_size=1)
        self.desired_landing_pub = rospy.Publisher('/desired_landing', PositionTarget, queue_size=1)
        
        # controller frequency in Hz
        self.hz = 20.0
        # Limit the rate in which ROS nodes run
        self.rate = rospy.Rate(self.hz)
        self.dt = (1.0 / self.hz)

        # PID controller class 
        self.pid_x = PID(kp=0.3, kd=0.05, ki=0.0, dt = self.dt)
        self.pid_y = PID(kp=0.3, kd=0.05, ki=0.0, dt = self.dt)

        # Desired orientation
        self.roll = 0
        self.pitch = 0
        self.yaw = 90*math.pi/180

        # Initialization
        self.relative_dis.data = [0, 0, 0]
    
    def mission_cb(self, msg):
        self.mission = msg.data

    def relative_dis_cb(self, msg):
        self.relative_dis = msg

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def calc_distance(self, x_d, y_d, z_d):
        return sqrt(pow((x_d), 2) + pow((y_d), 2) + pow((z_d), 2))
     

    def safety_landing(self, e):
        '''
        1) The value coming from the aurco_VIO node is nan:
            Position control is performed.
            It means that aruco marker is not recognized.
        2) The valuse is "not" nan:
            Velocity control is performed through PID control.
            It means that aruco marker is recognized.
        '''
        if self.mission == 6:
            if np.isnan(self.relative_dis.data[0]):
                qx, qy, qz, qw = math_utils.to_quaternion(self.yaw, 0, 0)
                self.desired_landing_position.pose.position.x = 0
                self.desired_landing_position.pose.position.y = 0
                self.desired_landing_position.pose.position.z = self.current_pose.pose.position.z - 0.1
                self.desired_landing_position.pose.orientation.x = qx
                self.desired_landing_position.pose.orientation.y = qy
                self.desired_landing_position.pose.orientation.z = qz
                self.desired_landing_position.pose.orientation.w = qw
                self.desired_landing_position_pub.publish(self.desired_landing_position)
            else:
                err_x = self.relative_dis.data[0] - 0
                err_y = self.relative_dis.data[1] - 0
                err_z = self.relative_dis.data[2] - 0
                err = self.calc_distance(err_x, err_y, err_z)

                #Compute PID
                vx = self.pid_x.compute(err_x)
                vy = self.pid_y.compute(err_y)

                self.desired_landing.yaw = self.yaw
                self.desired_landing.velocity.x = -vx*0.1
                self.desired_landing.velocity.y = -vy*0.1
                self.desired_landing.velocity.z = -0.1

                self.desired_landing_pub.publish(self.desired_landing)
            
            '''
            Function to automatically run a service call. 
            This function is defined in "koreauav_utils" package, and is executed if ros parameter("/srv_mode") is false in the waypoint_server.launch file.
            Mission num 11:
                AutoLanding -> Disarming
            '''
            if self.current_pose.pose.position.z < 0.3:
                auto_service.call_drone_command(11)


if __name__ == "__main__":
    rospy.init_node("PID_control_node")
    try:
        PID_control_node_handler = PIDControl()

        rate = rospy.Rate(100)
        # wait for FCU connection
        while not rospy.is_shutdown() and not PID_control_node_handler.current_state.connected:
            rate.sleep()
        rospy.loginfo("PID_control_node : FCU connected")

        rospy.Timer(rospy.Duration(0.05), PID_control_node_handler.safety_landing)
    
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


