#! usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import PoseStamped, PointStamped
from mavros_msgs.msg import State
from ysdrone_msgs.srv import *
from std_msgs.msg import Float32
from koreauav_utils import auto_service

class PathClass(object):
    def __init__(self):
        self.destination_cnt = 0
        self.destination_cnt_msg = Float32()
        
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.destination_cmd = PoseStamped()
        
        self.init_destination_cmd = PoseStamped()
        self.init_destination_check = False

        self.destination_1_pose = PointStamped()
        self.destination_2_pose = PointStamped()
        self.destination_3_pose = PointStamped()

        self.mission = 0

        #Subscriber
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)

        if rospy.has_param('/destination_z'):
            self.destination_z = rospy.get_param('/destination_z')
        else:
            self.destination_z = 3.0
            rospy.set_param('/destination_z', self.destination_z)
        rospy.logwarn(f"Check Z param! {rospy.get_param('/destination_z')}")
        rospy.logwarn(f"Check Z param! {rospy.get_param('/destination_z')}")
            
        self.waypoint_1_sub = rospy.Subscriber('/WPT_1_enu', PointStamped, self.waypoint_1_cb)
        self.waypoint_2_sub = rospy.Subscriber('/WPT_2_enu', PointStamped, self.waypoint_2_cb)
        self.waypoint_3_sub = rospy.Subscriber('/WPT_3_enu', PointStamped, self.waypoint_3_cb)
        #Publisher
        self.destination_cmd_pub = rospy.Publisher('/destination_command', PoseStamped, queue_size=1)
        self.destination_cnt_pub = rospy.Publisher('/destination_cnt', Float32, queue_size=1)


    def mission_cb(self, msg):
        self.mission = msg.data

    def state_cb(self, msg):
        self.current_state = msg
    
    def pose_cb(self, msg):
        self.current_pose = msg
        if not self.init_destination_check:
            self.init_destination_cmd.pose.position.x = self.current_pose.pose.position.x
            self.init_destination_cmd.pose.position.y = self.current_pose.pose.position.y
            self.init_destination_check = True
            
    def waypoint_1_cb(self, msg):
        self.destination_1_pose = msg
        
        rospy.set_param('/destination_1_pose_x', self.destination_1_pose.point.x)
        rospy.set_param('/destination_1_pose_y', self.destination_1_pose.point.y)
        
    def waypoint_2_cb(self, msg):
        self.destination_2_pose = msg
        
        rospy.set_param('/destination_2_pose_x', self.destination_2_pose.point.x)
        rospy.set_param('/destination_2_pose_y', self.destination_2_pose.point.y)
        
    def waypoint_3_cb(self, msg):
        self.destination_3_pose = msg
        
        rospy.set_param('/destination_3_pose_x', self.destination_3_pose.point.x)
        rospy.set_param('/destination_3_pose_y', self.destination_3_pose.point.y)
        

    def calc_xy_err(self, cur, dest):
        xy_err = math.sqrt((cur.pose.position.x - dest.pose.position.x)**2 + (cur.pose.position.y - dest.pose.position.y)**2)
        return xy_err

    def calc_z_err(self, cur, dest):
        z_err = math.sqrt((cur.pose.position.z - dest.pose.position.z)**2)
        return z_err
    
    def destination_publisher(self, e):
        if self.init_destination_check:
            self.destination_cmd.header.stamp = rospy.get_rostime()
            self.destination_cnt_msg.data = self.destination_cnt

            #=====================================LOCAL COORDINATE=======================================================
            self.destination_positions = [
                (self.destination_1_pose.point.x, self.destination_1_pose.point.y, self.destination_z),
                (self.destination_2_pose.point.x, self.destination_2_pose.point.y, self.destination_z),
                (self.destination_3_pose.point.x, self.destination_3_pose.point.y, self.destination_z)
            ]
            #rospy.loginfo(f"Waypoint 1 - x: {self.destination_1_pose.point.x}, y: {self.destination_1_pose.point.y}, z: {self.destination_z}")
            #rospy.loginfo(f"Waypoint 2 - x: {self.destination_2_pose.point.x}, y: {self.destination_2_pose.point.y}, z: {self.destination_z}")
            #rospy.loginfo(f"Waypoint 3 - x: {self.destination_3_pose.point.x}, y: {self.destination_3_pose.point.y}, z: {self.destination_z}")
            #=============================================================================================================

            if self.destination_cnt < len(self.destination_positions):
                self.destination_cmd.pose.position.x, self.destination_cmd.pose.position.y, self.destination_cmd.pose.position.z = self.destination_positions[self.destination_cnt]
                self.destination_cmd_pub.publish(self.destination_cmd)
                self.destination_cnt_pub.publish(self.destination_cnt_msg)
            else:
                if self.mission == 1:
                    auto_service.call_drone_command(2)


            if self.calc_xy_err(self.destination_cmd, self.current_pose) < 0.3 and self.calc_z_err(self.destination_cmd, self.current_pose) < 0.2:
                # TODO 대회에서 요구하는 정확도 확인 && 실제로 어느정도 정확하게 나오는지 확인 필요.
                self.destination_cnt += 1
                # 여기 나중에 수정 필요
                # if self.destination_cnt > 4:
                #     self.destination_cnt = 0
            

if __name__ == "__main__":
    rospy.init_node('path_node', anonymous=True)
    try:
        path_node_handler = PathClass()

        rate = rospy.Rate(100)
        # wait for FCU connection
        while not rospy.is_shutdown() and not path_node_handler.current_state.connected:
            rate.sleep()
        rospy.loginfo("Path node : FCU connected")
        # rospy.logwarn(f"Check Z value! {path_node_handler.destination_z}")
        # rospy.logwarn(f"Check Z value! {path_node_handler.destination_z}")

        rospy.Timer(rospy.Duration(0.05), path_node_handler.destination_publisher)

        rospy.spin()
    except rospy.ROSInterruptException as exception:
        pass
            
            
                


    


