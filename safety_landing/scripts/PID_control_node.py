#!/usr/bin/env python3
import rospy
import tf
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from math import pow, atan2, sqrt, pi, degrees
from std_msgs.msg import Float32MultiArray

class PID:
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
    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.relative_dis = Float32MultiArray()
        self.landing_velocity = Twist()
        self.tolerance_position = 0.01
        self.desired_landing = PositionTarget()

        #Subscriber
        self.relative_dis_sub = rospy.Subscriber("/relative_distance", Float32MultiArray, self.relative_dis_cb)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)

        #Publisher
        # Send the desired velocity of UAV to control_node in offboard package.
        self.desired_landing_pub = rospy.Publisher('/desired_landing', PositionTarget, queue_size=1)
        
        # controller frequency in Hz
        self.hz = 20.0
        # Limit the rate in which ROS nodes run
        self.rate = rospy.Rate(self.hz)
        self.dt = (1.0 / self.hz)

        # PID controller class 
        self.pid_x = PID(kp=1, dt = self.dt)
        self.pid_y = PID(kp=1, dt = self.dt)

        # Desired orientation
        self.roll = 0
        self.pitch = 0
        self.yaw = 90*math.pi/180

        # Initialization
        self.relative_dis.data = [0, 0, 0]
    
    def relative_dis_cb(self, msg):
        self.relative_dis = msg
        #rospy.loginfo(f"{self.relative_dis.data[0]}, {self.relative_dis.data[1]}, {self.relative_dis.data[2]}")

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def calc_distance(self, x_d, y_d, z_d):
        return sqrt(pow((x_d), 2) + pow((y_d), 2) + pow((z_d), 2))
     

    def safety_landing(self, e):
        try:
            err_x = self.relative_dis.data[0] - 0
            err_y = self.relative_dis.data[1] - 0
            err_z = self.relative_dis.data[2] - 0
            err = self.calc_distance(err_x, err_y, err_z)
        except IndexError:
            self.desired_landing.yaw = self.yaw
            self.desired_landing.velocity.x = 0
            self.desired_landing.velocity.y = 0
            self.desired_landing.velocity.z = -0.2
            self.desired_landing_pub.publish(self.desired_landing)
            rospy.loginfo("Warning: self.relative_dis.data has less than 3 elements. Skipping this cycle")
            return

        #Compute PID
        vx = self.pid_x.compute(err_x)
        vy = self.pid_y.compute(err_y)

        self.desired_landing.yaw = self.yaw
        self.desired_landing.velocity.x = -vx*0.8
        self.desired_landing.velocity.y = -vy*0.8
        self.desired_landing.velocity.z = -0.2

        self.desired_landing_pub.publish(self.desired_landing)

        #Debugging
        #rospy.loginfo(f"err_x: {err_x}, vx: {vx} || err_y: {err_y}, vy: {vy} || err_z: {err_z}, vz: {vz}")
        self.rate.sleep()


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


