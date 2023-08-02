#! /usr/bin/env python3
import rospy

from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped


class SetmodeClass(object):
    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.service_timeout = 30

        #Subscriber
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        
        self.target_pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        #Service
        rospy.loginfo('-------------Waiting for services to connect--------------')
        try:
            rospy.wait_for_service("/mavros/cmd/arming", self.service_timeout)
            rospy.wait_for_service('/mavros/set_mode', self.service_timeout)
        except rospy.ROSException as e:
            rospy.logerr('Failed to initialize service')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    def pose_cb(self, msg):
        self.current_pose = msg

    def state_cb(self, msg):
        prev_state = self.current_state
        self.current_state = msg

        if self.current_state.mode != prev_state.mode:
            rospy.loginfo(f"Current Mode : {self.current_state.mode}")
        if self.current_state.armed != prev_state.armed:
            rospy.loginfo(f"Vehicle armed : {self.current_state.armed}")

    def setMode(self, mode):
        rospy.logerr('Mode Changed')
        rate = rospy.Rate(10)
        for _ in range(20):
            self.target_pose.pose.position.x = 0
            self.target_pose.pose.position.y = 0
            self.target_pose.pose.position.z = 0
            self.target_pose_pub.publish(self.target_pose)
            rate.sleep()
        try:
            response = self.set_mode_client(base_mode = 0, custom_mode = mode)
            # return response.mode_sent
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed : %s" %e)

    def setArm(self):
        rate = rospy.Rate(0.5)
        while True:
            if self.current_state.armed is not True:
                self.arming_client(True)
            else:
                break
            rate.sleep()

    def check_FCU_connection(self):
        while not self.current_state.connected:
            rospy.loginfo_throttle(1, "Wait FCU connection")
        rospy.loginfo("FCU connected")

    def land(self):
        try:
            response = self.set_mode_client(base_mode = 0, custom_mode = "LAND")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed : %s" %e)
        rospy.loginfo("Landing...")
        # wait until the drone is disarmed
        while self.current_state.armed:
            rospy.sleep(1)
            rospy.loginfo("Disarming...")
        rospy.loginfo("Landed")


if __name__ == "__main__":
    rospy.init_node('setmode_node', anonymous=True)
    try:
        setmode_node_handler = SetmodeClass()
        setmode_node_handler.check_FCU_connection()
        setmode_node_handler.setArm()
        setmode_node_handler.setMode("OFFBOARD")
        rospy.loginfo("Ready to go")
        rospy.spin()
    except rospy.ROSInterruptException as exception:
        pass






