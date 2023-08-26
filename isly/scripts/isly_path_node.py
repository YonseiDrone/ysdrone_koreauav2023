#! usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import PoseStamped, PointStamped
from mavros_msgs.msg import State
from ysdrone_msgs.srv import *
from std_msgs.msg import Float32
from koreauav_utils import auto_service

class IslyPath(object):
    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.isly_destination = PoseStamped()

        self.init_destination_cmd = PoseStamped()
        self.init_destination_check = False

        self.destination_1_pose = PointStamped()
        self.destination_2_pose = PointStamped()
        self.destination_3_pose = PointStamped()

        self.destination_cnt = 0
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
        rospy.logwarn(f"Check Z value! {rospy.get_param('/destination_z')}")
        rospy.logwarn(f"Check Z value! {rospy.get_param('/destination_z')}")

        self.waypoint_1_sub = rospy.Subscriber('/WPT_1_enu', PointStamped, self.waypoint_1_cb)
        self.waypoint_2_sub = rospy.Subscriber('/WPT_2_enu', PointStamped, self.waypoint_2_cb)
        self.waypoint_3_sub = rospy.Subscriber('/WPT_3_enu', PointStamped, self.waypoint_3_cb)
        
        #Publisher
        self.isly_destination_pub = rospy.Publisher('/isly_destination_command', PoseStamped, queue_size=50)
    
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
        if self.init_destination_check and self.mission == 5:
            self.isly_destination.header.stamp = rospy.get_rostime()

            #=====================================LOCAL COORDINATE=======================================================
            self.destination_positions = [
                (self.destination_3_pose.point.x, self.destination_3_pose.point.y, self.destination_z),
                (self.destination_2_pose.point.x, self.destination_2_pose.point.y, self.destination_z),
                (self.destination_1_pose.point.x, self.destination_1_pose.point.y, self.destination_z)
            ]
            #rospy.loginfo(f"Waypoint 1 - x: {self.destination_1_pose.point.x}, y: {self.destination_1_pose.point.y}, z: {self.destination_z}")
            #rospy.loginfo(f"Waypoint 2 - x: {self.destination_2_pose.point.x}, y: {self.destination_2_pose.point.y}, z: {self.destination_z}")
            #rospy.loginfo(f"Waypoint 3 - x: {self.destination_3_pose.point.x}, y: {self.destination_3_pose.point.y}, z: {self.destination_z}")
            #==============================================================================================================

            if self.destination_cnt < len(self.destination_positions):
                self.isly_destination.pose.position.x, self.isly_destination.pose.position.y, self.isly_destination.pose.position.z = self.destination_positions[self.destination_cnt]
                self.isly_destination_pub.publish(self.isly_destination)
            else:
                if self.mission == 5:
                    auto_service.call_drone_command(9)

            if self.calc_xy_err(self.isly_destination, self.current_pose) < 0.3 and self.calc_z_err(self.isly_destination, self.current_pose) < 0.2:
                self.destination_cnt += 1
                # 여기 나중에 수정 필요
                # if self.destination_cnt > 4:
                #     self.destination_cnt = 0
            
            self.isly_destination_pub.publish(self.isly_destination)


if __name__ == "__main__":
    rospy.init_node('isly_path_node', anonymous=True)
    try:
        isly_path_node_handler = IslyPath()

        rate = rospy.Rate(100)
        # wait for FCU connection
        while not rospy.is_shutdown() and not isly_path_node_handler.current_state.connected:
            rate.sleep()
        rospy.loginfo("Isly Path node : FCU connected")

        rospy.Timer(rospy.Duration(0.05), isly_path_node_handler.destination_publisher)

        rospy.spin()
    except rospy.ROSInterruptException as exception:
        pass

    






