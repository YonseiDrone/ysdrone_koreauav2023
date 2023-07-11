#!/usr/bin/env python3
import rospy
import tf
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
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
        self.err_previous = 0.001
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

        #Subscriber
        self.relative_dis_sub = rospy.Subscriber("/relative_distance", Float32MultiArray, self.relative_dis_cb)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)

        #Publisher
        # Send the desired velocity of UAV to control_node in offboard package.
        self.landing_vel_pub = rospy.Publisher("/landing_velocity", Twist, queue_size=1)

        # controller frequency in Hz
        self.hz = 20.0
        # Limit the rate in which ROS nodes run
        self.rate = rospy.Rate(self.hz)
        self.dt = (1.0 / self.hz)

        # PID controller class 
        self.pid_x = PID(kp=1, dt = self.dt)
        self.pid_y = PID(kp=1, dt = self.dt)
        self.pid_z = PID(kp=1, dt = self.dt)
    
    def relative_dis_cb(self, msg):
        self.relative_dis = msg
        #rospy.loginfo(f"{self.relative_dis.data[0]}, {self.relative_dis.data[1]}, {self.relative_dis.data[2]}")

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def calc_distance(self, x_d, y_d, z_d):
        return sqrt(pow((x_d), 2) + pow((y_d), 2) + pow((z_d), 2))
     

    def safety_landing(self):
        while True:
            if len(self.relative_dis.data) >=3:
                err_x = self.relative_dis.data[0] - 0
                err_y = self.relative_dis.data[1] - 0
                err_z = self.relative_dis.data[2] - 0
                err = self.calc_distance(err_x, err_y, err_z)
                break
        
        while (err >= self.tolerance_position or self.current_pose.pose.position.z > 0.5):
            #rospy.loginfo(f"Distance form goal: {err}")

            err_x = self.relative_dis.data[0] - 0
            err_y = self.relative_dis.data[1] - 0
            err_z = self.relative_dis.data[2] - 0
            err = self.calc_distance(err_x, err_y, err_z)

            #Compute PID
            vx = self.pid_x.compute(err_x)
            vy = self.pid_y.compute(err_y)
            vz = self.pid_z.compute(err_z)

            self.landing_velocity.linear.x = -vx
            self.landing_velocity.linear.y = -vy
            #self.landing_velocity.linear.z = -vz * 0.1
            self.landing_velocity.angular.x = 0.0
            self.landing_velocity.angular.y = 0.0
            self.landing_velocity.angular.z = 0.0

            #Debugging
            #rospy.loginfo(f"err_x: {err_x}, vx: {vx} || err_y: {err_y}, vy: {vy} || err_z: {err_z}, vz: {vz}")
            self.landing_vel_pub.publish(self.landing_velocity)
            self.rate.sleep()
        
        # 0<z<0.5인 경우(즉, 거리가 가까워서 이미지가 안보이는 경우)
        if self.current_pose.pose.position.z > 0:
            rospy.loginfo("Landing Begin")
            self.landing_velocity.linear.x = 0.0
            self.landing_velocity.linear.y = 0.0
            self.landing_velocity.linear.z = -1.0

            for i in range(10):
                self.landing_vel_pub.publish(self.landing_velocity)
                self.rate.sleep()

        # Stop the drone
        self.landing_velocity.linear.x = 0.0
        self.landing_velocity.linear.y = 0.0
        self.landing_velocity.linear.z = 0.0
        self.landing_vel_pub.publish(self.landing_velocity)

        #Land
        rospy.loginfo("\n Landing")
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            response = land_client(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Landing failed: %s" %e)

        # Disarm
        print("\n Disarming")
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arming_client(value = False)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Disarming failed: %s" %e)


if __name__ == "__main__":
    rospy.init_node("PID_control_node")
    try:
        PID_control_node_handler = PIDControl()

        rate = rospy.Rate(100)
        # wait for FCU connection
        while not rospy.is_shutdown() and not PID_control_node_handler.current_state.connected:
            rate.sleep()
        rospy.loginfo("PID_control_node : FCU connected")

        PID_control_node_handler.safety_landing()
    
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


