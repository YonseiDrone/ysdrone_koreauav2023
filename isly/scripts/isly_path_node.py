#! usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

class IslyPath(object):
    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.isly_destination = PoseStamped()
        self.init_destination_cmd = PoseStamped()
        self.init_destination_check = False
        self.destination_cnt = 0

        #Subscriber
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        #Publisher
        self.isly_destination_pub = rospy.Publisher('/isly_destination_command', PoseStamped, queue_size=50)
    def state_cb(self, msg):
        self.current_state = msg
    
    def pose_cb(self, msg):
        self.current_pose = msg
        if not self.init_destination_check:
            self.init_destination_cmd.pose.position.x = self.current_pose.pose.position.x
            self.init_destination_cmd.pose.position.y = self.current_pose.pose.position.y
            self.init_destination_check = True
    def calc_xy_err(self, cur, dest):
        xy_err = math.sqrt((cur.pose.position.x - dest.pose.position.x)**2 + (cur.pose.position.y - dest.pose.position.y)**2)
        return xy_err

    def calc_z_err(self, cur, dest):
        z_err = math.sqrt((cur.pose.position.z - dest.pose.position.z)**2)
        return z_err

    def destination_publisher(self, e):
        if self.init_destination_check:
            self.time_now = rospy.get_rostime()

            self.isly_destination.header.stamp = self.time_now

            self.destination_positions = [
                (10.0, 8.0, 4.0),
                (0.0, 5.0, 4.0),
                (0.0, 0.0, 4.0)
            ]

            self.isly_destination.pose.position.x, self.isly_destination.pose.position.y, self.isly_destination.pose.position.z = self.destination_positions[self.destination_cnt]

            if self.calc_xy_err(self.isly_destination, self.current_pose) < 0.3 and self.calc_z_err(self.isly_destination, self.current_pose) < 0.2:
                self.destination_cnt += 1
                # 여기 나중에 수정 필요
                if self.destination_cnt > 4:
                    self.destination_cnt = 0
            
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

    






