#! /usr/bin/env python3

import rospy, math
from ysdrone_msgs.srv import *
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32, Header
from sensor_msgs.msg import PointCloud2, Imu, PointField
import sensor_msgs.point_cloud2 as pc2
import pcl
import numpy as np
from enum import Enum
from koreauav_utils import auto_service

class Mission(Enum):
    TAKEOFF = 0
    AVOIDANCE = 1
    BUILDING_SEARCH = 2
    MARKER_APPROACH = 3

### Util functions
def to_quaternion(yaw, pitch, roll):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    qw = cr*cp*cy + sr*sp*sy

    return qx, qy, qz, qw

def to_euler_angles(x, y, z, w):
    # roll(x-axis rotation)
    sinr_cosp = 2 * (w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    angles_roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch(y-axis rotation)
    sinp = 2*(w*y - z*x)
    if abs(sinp) >= 1:
        angles_pitch = math.copysign(math.pi/2, sinp) # use 90 degrees if out of range
    else:
        angles_pitch = math.asin(sinp)
    
    # yaw(z-axis rotation)
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    angles_yaw = math.atan2(siny_cosp, cosy_cosp)

    return angles_roll, angles_pitch, angles_yaw

### Class
class BuildingSearch(object):
    def __init__(self):
        self.initAngle = False
        self.current_pose = PoseStamped()
        self.current_state = State()
        self.target_pose = PoseStamped()
        self.centroid = PoseStamped()
        self.mission = Float32()
        self.imu = Imu()
        self.building_centroid = []
        self.pointcloud_centroid = None
        self.current_angle = 0.0

        # ROS params
        self.srv_mode = rospy.get_param("/srv_mode", True)
        self.last_goal_x = rospy.get_param("/destination_3_pose_x", 80.0)
        self.last_goal_y = rospy.get_param("/destination_3_pose_y", -41.0)
        self.last_goal_z = rospy.get_param("/destination_z", 3)
        self.search_height = rospy.get_param('search_height', 3)
        self.BUILDING_SEARCH_COUNT = rospy.get_param("building_search_count", 15)
        self.BUILDING_STACK_COUNT = rospy.get_param("building_stack_count", 15)
        self.radius = rospy.get_param("building_search_radius", 1.0)
        self.speed = rospy.get_param("building_search_speed", 0.1)

        # ROS publisher & subscriber
        self.cloud_sub = rospy.Subscriber('/local_pointcloud', PointCloud2, self.cloud_cb)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)

        self.target_pose_pub = rospy.Publisher('/building/search/target_pose',PoseStamped, queue_size=1)
        self.centroid_pub = rospy.Publisher('/building/search/centroid_pose', PoseStamped, queue_size=1)
        self.cluster_visual_pub = rospy.Publisher('/cluster_pointcloud', PointCloud2, queue_size=1)
    
    def publish_pointcloud(self, points):
        """Publish the topic in 'PointCloud2' type to visualize in RViz

        Args:
            points: XYZ array of PointCloud()
        """
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'local_origin'

        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.cluster_visual_pub.publish(cloud_msg)

    ## Callback functions
    def imu_cb(self, msg):
        self.imu = msg

    def mission_cb(self, msg):
        self.mission = msg.data

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def cloud_cb(self, input):
        # Read points from ROS PointCloud2 message
        points_generator = pc2.read_points(input, skip_nans=True, field_names=("x", "y", "z"))
        # Convert the generator to a list of tuples
        points_list = [tuple(point) for point in points_generator if point[2]>0.5]

        # Create a pcl.PointCloud object
        cloud = pcl.PointCloud()
        cloud.from_list(points_list)
        if self.mission in [Mission.BUILDING_SEARCH.value, Mission.MARKER_APPROACH.value]:
            ### Find the centroid of clustered object and Append to the 'building_centroid' list
            if cloud.size > 0:
                tree = cloud.make_kdtree()
                ec = cloud.make_EuclideanClusterExtraction()
                ec.set_ClusterTolerance(4.0)
                ec.set_MinClusterSize(30)
                ec.set_MaxClusterSize(600)
                ec.set_SearchMethod(tree)
                cluster_indices = ec.Extract()

                for _, indices in enumerate(cluster_indices):
                    cluster_points = [] # List to hold the cluster points

                    for i in indices:
                        point = cloud[i]
                        cluster_points.append(point)
                        
                    # Create pcl.PointCloud objects for the cluster and colored cluster
                    cloud_cluster = pcl.PointCloud()
                    cloud_cluster.from_list(cluster_points)
                    self.publish_pointcloud(cloud_cluster.to_array())
                    
                    self.pointcloud_centroid = np.mean(cloud_cluster.to_array(), axis=0)
                    self.building_centroid.append(self.pointcloud_centroid)
            else:
                rospy.logwarn("Empty input cloud!")

            ### Since there are too many noises, Ignore centroids collected before the count 
            if len(self.building_centroid) < self.BUILDING_SEARCH_COUNT:
                
                if self.initAngle is False:
                    _, _, self.current_angle = to_euler_angles(self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w)
                    self.initAngle = True
                
                ### Navigate in a circle from the WPT#3
                self.target_pose.pose.position.x = self.current_pose.pose.position.x + 0.5 * (self.last_goal_x + self.radius * math.cos(self.current_angle) - self.current_pose.pose.position.x)
                self.target_pose.pose.position.y = self.current_pose.pose.position.y + 0.5 * (self.last_goal_y + self.radius * math.sin(self.current_angle) - self.current_pose.pose.position.y)
                self.target_pose.pose.position.z = self.search_height
                
                qx, qy, qz, qw = to_quaternion(self.current_angle, 0, 0)
                self.target_pose.pose.orientation.x = qx
                self.target_pose.pose.orientation.y = qy
                self.target_pose.pose.orientation.z = qz
                self.target_pose.pose.orientation.w = qw
                
                self.target_pose_pub.publish(self.target_pose)

                self.current_angle += self.speed
            
            elif self.BUILDING_SEARCH_COUNT <= len(self.building_centroid) <= self.BUILDING_SEARCH_COUNT + self.BUILDING_STACK_COUNT:
                self.target_pose.pose.position.x = self.last_goal_x + self.radius * math.cos(self.current_angle)
                self.target_pose.pose.position.y = self.last_goal_y + self.radius * math.sin(self.current_angle)
                self.target_pose.pose.position.z = self.search_height
                
                error_yaw = math.atan2(self.pointcloud_centroid[1]-self.current_pose.pose.position.y, self.pointcloud_centroid[0]-self.current_pose.pose.position.x)
                qz = math.sin(error_yaw / 2.0)
                qw = math.cos(error_yaw / 2.0)
                self.target_pose.pose.orientation.x = 0
                self.target_pose.pose.orientation.y = 0
                self.target_pose.pose.orientation.z = qz
                self.target_pose.pose.orientation.w = qw
                
                self.target_pose_pub.publish(self.target_pose)

            ### After collecting enough points, control the heading to the target
            else:
                self.target_pose.pose.position.x = self.last_goal_x
                self.target_pose.pose.position.y = self.last_goal_y
                self.target_pose.pose.position.z = self.search_height
                
                error_yaw = math.atan2(self.pointcloud_centroid[1]-self.current_pose.pose.position.y, self.pointcloud_centroid[0]-self.current_pose.pose.position.x)
                qz = math.sin(error_yaw/2.0)
                qw = math.cos(error_yaw/2.0)
                self.target_pose.pose.orientation.x = 0
                self.target_pose.pose.orientation.y = 0
                self.target_pose.pose.orientation.z = qz
                self.target_pose.pose.orientation.w = qw
                
                self.target_pose_pub.publish(self.target_pose)

                centroid = np.mean(np.array(self.building_centroid[-29:]), axis=0)  # Assume that some points are noisy, use the most recent centroid points
                self.centroid.pose.position.x = centroid[0]
                self.centroid.pose.position.y = centroid[1]
                self.centroid.pose.position.z = centroid[2]
                self.centroid_pub.publish(self.centroid)

                if len(self.building_centroid) == self.BUILDING_SEARCH_COUNT+self.BUILDING_STACK_COUNT+10:
                    auto_service.call_drone_command(3)


if __name__ == "__main__":
    rospy.init_node('building_search_python_node', anonymous=True)
    try:
        building_search_node = BuildingSearch()

        rate = rospy.Rate(100)
        # wait for FCU connection
        while not rospy.is_shutdown() and not building_search_node.current_state.connected:
            rate.sleep()
        rospy.loginfo("Building Search Python Node : FCU connected")
        rospy.spin()
    except rospy.ROSInitException as e:
        pass