import rospy, math
from ysdrone_msgs.srv import *
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs.point_cloud2 as pc2
import pcl
import numpy as np

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

class BuildingSearch(object):
    def __init__(self):
        self.current_pose = PoseStamped()
        self.current_state = State()
        self.target_pose = PoseStamped()
        self.centroid = PoseStamped()
        self.mission = Float32()
        self.imu = Imu()
        self.searching_status = 0
        self.building_centroid = []
        self.pointcloud_centroid = None

        # ROS params
        self.srv_mode = rospy.get_param("/srv_mode", True)
        self.building_search_mission = rospy.get_param("/building_search_mission", 2.0)
        self.last_goal_x = rospy.get_param("/destination_3_pose_x", 65.0)
        self.last_goal_y = rospy.get_param("/destination_3_pose_y", -43.0)
        self.last_goal_z = rospy.get_param("/destination_z", 15)

        # ROS publisher & subscriber
        self.cloud_sub = rospy.Subscriber('/local_pointcloud', PointCloud2, self.cloud_cb)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)

        self.target_pose_pub = rospy.Publisher('/building/search/target_pose',PoseStamped, queue_size=1)
        self.centroid_pub = rospy.Publisher('/building/search/centroid_pose', PoseStamped, queue_size=1)

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
        points_list = [tuple(point) for point in points_generator]

        # Create a pcl.PointCloud object
        cloud = pcl.PointCloud()
        cloud.from_list(points_list)
        colored_cloud = pcl.PointCloud()
        if self.mission == 2:
            if cloud.size > 0:
                tree = cloud.make_kdtree()
                ec = cloud.make_EuclideanClusterExtraction()
                ec.set_ClusterTolerance(4.0)
                ec.set_MinClusterSize(30)
                ec.set_MaxClusterSize(400)
                ec.set_SearchMethod(tree)
                cluster_indices = ec.Extract()

                for j, indices in enumerate(cluster_indices):
                    cluster_points = [] #List to hold the cluster points

                    for i in indices:
                        point = cloud[i]
                        cluster_points.append(point)
                        
                    
                    # Create pcl.PointCloud objects for the cluster and colored cluster
                    cloud_cluster = pcl.PointCloud()
                    cloud_cluster.from_list(cluster_points)
                    

                    self.pointcloud_centroid = np.mean(cloud_cluster.to_array(), axis=0)
                    rospy.loginfo(f"pointcloud_centroid: {self.pointcloud_centroid}")
                    self.building_centroid.append(self.pointcloud_centroid)

                    # Calculate the dimensionis of the bounding box
                    min_pt = np.min(cloud_cluster.to_array(), axis=0)
                    max_pt = np.max(cloud_cluster.to_array(), axis=0)

                    # # Create and publish message
                    # self.target_pose.pose.position.x = (centroid[0] + self.last_goal_x) / 2
                    # self.target_pose.pose.position.y = (centroid[1] + self.last_goal_y) / 2
                    # self.target_pose.pose.position.z = max(centroid[2], self.last_goal_z)
                    # self.target_pose_pub.publish(self.target_pose)

                if len(self.building_centroid) < 15:
                    self.target_pose.pose.position.x = self.last_goal_x
                    self.target_pose.pose.position.y = self.last_goal_y
                    self.target_pose.pose.position.z = 9
                    roll, pitch, yaw = to_euler_angles(self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w)
                    target_yaw = yaw - 0.2
                    qx, qy, qz, qw = to_quaternion(target_yaw, pitch, roll)
                    self.target_pose.pose.orientation.x = qx
                    self.target_pose.pose.orientation.y = qy
                    self.target_pose.pose.orientation.z = qz
                    self.target_pose.pose.orientation.w = qw
                    self.target_pose_pub.publish(self.target_pose)
                
                elif 15<=len(self.building_centroid)<=30:
                    rospy.loginfo("No 2.")
                    self.target_pose.pose.position.x = self.last_goal_x
                    self.target_pose.pose.position.y = self.last_goal_y
                    self.target_pose.pose.position.z = 9
                    error_yaw = math.atan2(self.pointcloud_centroid[1]-self.current_pose.pose.position.y, self.pointcloud_centroid[0]-self.current_pose.pose.position.x)
                    qz = math.sin(error_yaw/2.0)
                    qw = math.cos(error_yaw/2.0)
                    self.target_pose.pose.orientation.x = 0
                    self.target_pose.pose.orientation.y = 0
                    self.target_pose.pose.orientation.z = qz
                    self.target_pose.pose.orientation.w = qw
                    self.target_pose_pub.publish(self.target_pose)

                else:
                    rospy.loginfo("here")
                    self.target_pose.pose.position.x = self.last_goal_x
                    self.target_pose.pose.position.y = self.last_goal_y
                    self.target_pose.pose.position.z = 9
                    error_yaw = math.atan2(self.pointcloud_centroid[1]-self.current_pose.pose.position.y, self.pointcloud_centroid[0]-self.current_pose.pose.position.x)
                    qz = math.sin(error_yaw/2.0)
                    qw = math.cos(error_yaw/2.0)
                    self.target_pose.pose.orientation.x = 0
                    self.target_pose.pose.orientation.y = 0
                    self.target_pose.pose.orientation.z = qz
                    self.target_pose.pose.orientation.w = qw
                    self.target_pose_pub.publish(self.target_pose)
                    self.centroid.pose.position.x = self.pointcloud_centroid[0]
                    self.centroid.pose.position.y = self.pointcloud_centroid[1]
                    self.centroid.pose.position.z = self.pointcloud_centroid[2]
                    rospy.loginfo(f"centroid: {self.centroid}")
                    self.centroid_pub.publish(self.centroid)

            else:
                rospy.logwarn("Empty input cloud!")

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