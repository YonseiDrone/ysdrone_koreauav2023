#!/usr/bin/env python3

import math, time
import rospy, rospkg
import cv2, torch
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy.linalg import svd, lstsq
import pandas
import matplotlib.pyplot as plt

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from mavros_msgs.msg import State
from std_msgs.msg import Float32, String

from ysdrone_msgs.srv import *
from koreauav_utils import auto_service
import message_filters
from mpl_toolkits.mplot3d import Axes3D

from sklearn.preprocessing import RobustScaler
from sklearn.decomposition import PCA
from sklearn.linear_model import LinearRegression, SGDRegressor

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
    

class MarkerDetection(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.yoloPath = self.rospack.get_path('yolo_cross_detection') + '/yolov5'
        self.weightPath = self.rospack.get_path('yolo_cross_detection') + '/weight/yolov5nV4.onnx'
        self.model = torch.hub.load(self.yoloPath, 'custom', self.weightPath, source='local', force_reload=False)
        self.model.iou = 0.5
        self.mission = 0
        self.mission_rep = ""
        self.setpoint_list = []
        self.crosspos_list = []
        self.dronepos_list = []
        self.LR = LinearRegression()
        self.SGD = SGDRegressor(learning_rate='constant', eta0=1e-3)

        self.counter = 0
        self.id = 0

        #setpoint
        self.offset = rospy.get_param("yolo_offset") # Distance from the cross marker
        self.circular_speed = rospy.get_param("circular_speed")
        self.radius = rospy.get_param("radius")
        self.yolo_search_count = rospy.get_param("yolo_search_count")
        self.yolo_stack_count = rospy.get_param("yolo_stack_count")
        self.setpoint_criterion = rospy.get_param("setpoint_criterion")
        self.setpoint_count = rospy.get_param("setpoint_count")
        self.obstacle_bound = rospy.get_param("obstacle_bound")

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.final_coord = PoseStamped()
        self.imu = Imu()
        self.target_pose = PoseStamped()
        self.centroid = None
        self.centroid_list = []

        self.downward = None

        # Subscriber
        self.rgb_image_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.downward_sub = rospy.Subscriber('/cv_image', Image, self.downward_cb)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)
        self.mission_rep_sub = rospy.Subscriber('/mission_rep', String, self.mission_rep_cb)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)
        self.centroid_sub = rospy.Subscriber('/building/search/centroid_pose', PoseStamped, self.centroid_cb)

        # Synchronize the topics
        ts = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.image_cb)

        # Publisher
        self.image_pub = rospy.Publisher('/cross_image', Image, queue_size=10)
        self.approch_pub = rospy.Publisher('/cross_marker_approch_setpoint', Marker, queue_size=1)
        self.centroid_pub = rospy.Publisher('/building_centroid', Marker, queue_size=1)
        self.obstacle_bound_pub = rospy.Publisher('/obstacle_bound', Marker, queue_size=1)
        self.cross_marker_pub = rospy.Publisher('/cross_marker_position', Marker, queue_size=1)
        self.setpoint_pub = rospy.Publisher('/setpoint', Marker, queue_size=1)
        self.target_pose_pub = rospy.Publisher('/launch_setposition', PoseStamped, queue_size=1)

        # Parameter for Potential Field
        self.Kp_att = 0.1
        self.Kp_rel = 100.0

        self.intrinsic_matrix = np.array([[454.6857718666893, 0.0, 424.5],
                        [0.0, 454.6857718666893, 240.5],
                        [0.0, 0.0, 1.0]])

        rospy.on_shutdown(self.visualize)

    def log_matrices(self, cross_pos_3d, other_pos_3d, drone_pos_3d, setpoint, id=0):
        if id % 1 == 0:
            d = f'/home/yonsei/logs/'
            np.save(d + f'cross_pos_3d_{id}.npy', cross_pos_3d)
            np.save(d + f'other_pos_3d_{id}.npy', other_pos_3d)
            np.save(d + f'drone_pos_3d_{id}.npy', drone_pos_3d)
            np.save(d + f'setpoint_{id}.npy', setpoint)

    def centroid_cb(self, msg):
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        self.centroid = np.array([x,y,z])
        marker = self.make_cube_marker(self.centroid, (0, 255, 0), 0.6)
        self.centroid_pub.publish(marker)

    
    def visualize(self):
        rospy.loginfo("=================VISUAL================")
        fig = plt.figure()
        ax = fig.add_subplot(111,projection='3d')
        self.setpoint_list = np.array(self.setpoint_list)
        self.crosspos_list = np.array(self.crosspos_list)
        self.dronepos_list = np.array(self.dronepos_list)
        setpoint_x = self.setpoint_list[:, 0]
        setpoint_y = self.setpoint_list[:, 1]
        setpoint_z = self.setpoint_list[:, 2]
        cross_x = self.crosspos_list[:, 0]
        cross_y = self.crosspos_list[:,1]
        cross_z = self.crosspos_list[:,2]
        drone_x = self.dronepos_list[:, 0]
        drone_y = self.dronepos_list[:,1]
        drone_z = self.dronepos_list[:,2]
        df = pandas.DataFrame()
        df['setpoint_x'] = setpoint_x
        df['setpoint_y'] = setpoint_y
        df['setpoint_z'] = setpoint_z
        df['cross_x'] = cross_x
        df['cross_y'] = cross_y
        df['cross_z'] = cross_z
        df['drone_x'] = drone_x
        df['drone_y'] = drone_y
        df['drone_z'] = drone_z
        df.to_csv('~/Downloads/3d.csv')

    def mission_cb(self, msg):
        self.mission = msg.data

    def mission_rep_cb(self, msg):
        self.mission_rep = msg.data

    def imu_cb(self, msg):
        self.imu = msg

    def downward_cb(self, msg):
        bridge = CvBridge()
        self.downward = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def state_cb(self, msg):
        prev_state = self.current_state
        self.current_state = msg

        if self.current_state.mode != prev_state.mode:
            rospy.loginfo(f"Current Mode : {self.current_state.mode}")
        if self.current_state.armed != prev_state.armed:
            rospy.loginfo(f"Vehicle armed : {self.current_state.armed}")
    
    def pose_cb(self, msg):
        self.current_pose = msg

    def get_3d_coord_fast(self, pixels, depth_frame):
        pixels = np.array(pixels).astype(np.int32)
        distances = depth_frame[pixels[:, 1], pixels[:, 0]]

        # Remove NaN values from numpy array 
        depth_mean = np.mean(distances[~np.isnan(distances)])

        #=====================Image to Camera======================================================================
        camera_coords = np.ones((4, len(distances)))
        camera_coords[2, :] = distances
        camera_coords[0, :] = (pixels[:, 0] - self.intrinsic_matrix[0, 2]) * camera_coords[2, :] / self.intrinsic_matrix[0, 0] #x
        camera_coords[1, :] = (pixels[:, 1] - self.intrinsic_matrix[1, 2]) * camera_coords[2, :] / self.intrinsic_matrix[1, 1] #y
        #=====================Camera to FLU========================================================================
        camera_to_flu = np.array([[ 6.12323400e-17,  0.00000000e+00,  1.00000000e+00,  0.00000000e+00], 
                                [-1.00000000e+00,  6.12323400e-17,  6.12323400e-17,  0.00000000e+00], 
                                [-6.12323400e-17, -1.00000000e+00,  3.74939946e-33,  0.00000000e+00], 
                                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        flu_coords = np.dot(camera_to_flu, camera_coords)
        #======================FLU to ENU==========================================================================
        roll, pitch, yaw = to_euler_angles(self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w)

        enu_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        enu_roll = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        enu_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        enu_rotation = np.dot(enu_yaw, enu_pitch, enu_roll)
        enu_translation = np.array([0, 0, 0])
        flu_to_enu = np.eye(4)
        flu_to_enu[:3, :3] = enu_rotation
        flu_to_enu[:3, 3] = enu_translation
        enu_coords = np.dot(flu_to_enu, flu_coords)
        #=======================ENU to Local position==============================================================
        enu_coords[0, :] += self.current_pose.pose.position.x
        enu_coords[1, :] += self.current_pose.pose.position.y
        enu_coords[2, :] += self.current_pose.pose.position.z
        enu_coords = enu_coords.T

    # TODO: Instead of return `depth_mean`, Use get_depth_mean
        return enu_coords[:,0:3], depth_mean
    
    def get_2d_coord(self, position):
        # inverse tf of get_3d_coord
        #=============================Local to ENU coordinate(drone position)===============
        position = np.array([position[0], position[1], position[2], 1])
        enu_coord = np.ones(4)
        enu_coord[0] = position[0] - self.current_pose.pose.position.x
        enu_coord[1] = position[1] - self.current_pose.pose.position.y
        enu_coord[2] = position[2] - self.current_pose.pose.position.z

        #==================================ENU to FLU coordinate(east-north-up)=======================================
        _, _, yaw = to_euler_angles(self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w)
        yaw *= -1
        enu_rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        enu_translation = np.array([0, 0, 0])

        enu_to_flu = np.eye(4)
        enu_to_flu[:3, :3] = enu_rotation
        enu_to_flu[:3, 3] = enu_translation

        flu_coord = np.dot(enu_to_flu, enu_coord)

        #===============================FLU to Camera coordinate(front-left-up)==========================================
        x_rotation = -90 * math.pi /180
        y_rotation = 90 * math.pi /180
        z_rotation = 0 
        flu_x_rotation = np.array([[1,0,0], [0, np.cos(x_rotation), -np.sin(x_rotation)], [0, np.sin(x_rotation), np.cos(x_rotation)]])
        flu_y_rotation = np.array([[np.cos(y_rotation),0,np.sin(y_rotation)], [0,1,0], [-np.sin(y_rotation), 0, np.cos(y_rotation)]])
        flu_rotation = np.dot(flu_x_rotation, flu_y_rotation)
        flu_rotation = np.linalg.inv(flu_rotation)
        flu_translation = np.array([0, 0, 0])

        flu_to_cam = np.eye(4)
        flu_to_cam[:3, :3] = flu_rotation
        flu_to_cam[:3, 3] = flu_translation

        cam_coord = np.dot(flu_to_cam, flu_coord)
        #===============================Camera to Pixel coordinate==========================================
        cam_coord = cam_coord[:3]
        image_coordinates = np.dot(self.intrinsic_matrix, cam_coord)
        u = image_coordinates[0] / image_coordinates[2]
        v = image_coordinates[1] / image_coordinates[2]

        return [int(u), int(v)]
    
    
    def square_sampling(self, left_top, right_bottom, interval=5):
        result = []
        x_start, y_start = left_top
        x_end, y_end = right_bottom

        for x in range(x_start, x_end, interval):
            for y in range(y_start, y_end, interval):
                result.append((x, y))

        return result
    
    def cal_approch_setpoint(self, cross_pos, other_pos, drone_pos, offset, depth_mean):
        points = np.array(other_pos)

        mean_point = np.mean(points, axis=0)
        centered_points = points - mean_point
        # # =====SVD(Singular Value Decomposition)======
        _, _, vh = svd(centered_points)
        slist = []
        for vector in vh:
            s = np.sum(np.dot(centered_points, vector))
            slist.append(s)
        min_idx = np.argmin(slist)
        normal_vector = np.array([vh[min_idx][0], vh[min_idx][1], 0])
        # normal_vector = vh[-1]
        # rospy.loginfo(f"normal vector: {normal_vector}")
        # normal_vector = np.array([normal_vector[0], normal_vector[1], 0])
        #======================PCA===========================
        # cov_matrix = np.cov(centered_points[:, :2], rowvar=False)
        # eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        # normal_vector = eigenvectors[:, np.argmin(eigenvalues)]
        # normal_vector = np.array([normal_vector[0], normal_vector[1], 0])
        #================================================
        #========================LSTSQ======================
        # A = np.column_stack((other_pos[:, :2], np.ones(other_pos.shape[0])))
        # c, _, _, _ = lstsq(A, np.ones(other_pos.shape[0]))
        # normal_vector = np.array([c[0], c[1], 0])
        #======================================================
        #========================LR============================
        # self.LR.fit(points[:, 0].reshape(-1, 1), points[:, 1].reshape(-1, 1))
        # normal_vector = np.array([1, -1/self.LR.coef_[0][0], 0])
        #========================================================
        #========================SGD============================
        # self.SGD.fit(points[:, 0].reshape(-1, 1), points[:, 1])
        # normal_vector = np.array([1, self.SGD.coef_[0], 0])
        #========================================================

        cross_drone_vec = cross_pos - drone_pos
        if np.dot(normal_vector, cross_drone_vec) > 0:
            normal_vector = -normal_vector

        # drone setpoint
        # return cross_pos + normal_vector / np.linalg.norm(normal_vector) * (offset + (offset - depth_mean)*0.3), vh
        return cross_pos + normal_vector / np.linalg.norm(normal_vector) * offset

    def make_cube_marker(self, pos, color, scale):
        #visualize
        marker = Marker()
        marker.type = Marker.CUBE
        marker.header.frame_id = 'local_origin'
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.pose.position = Point(pos[0], pos[1], pos[2])

        return marker
    
    def make_line_marker(self, pos, radius):
        marker2 = Marker()
        marker2.lifetime = rospy.Duration()
        marker2.header.frame_id = "local_origin"
        marker2.type = Marker.LINE_STRIP
        marker2.action = Marker.ADD
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 1.0
        marker2.color.b = 1.0
        marker2.scale.x = 0.1

        points = []
        samples = 50
        for i in range(samples):
            x = pos[0] + radius * math.cos(math.pi * 2.0 * float(i) / float(samples))
            y = pos[1] + radius * math.sin(math.pi * 2.0 * float(i) / float(samples))
            z = pos[2]
            points.append(Point(x, y, z))
        marker2.points = points
        return marker2

    def calc_attractive_force(self, x, y, gx, gy):
        e_x, e_y = gx-x, gy-y
        distance = np.linalg.norm([e_x, e_y])

        self.Kp_att = distance * 0.15
        att_x = self.Kp_att * e_x/distance
        att_y = self.Kp_att * e_y/distance

        return att_x, att_y
    
    def calc_repulsive_force(self, x, y, obs):
        rep_x, rep_y = 0.0, 0.0

        for obs_xy in np.ndindex(obs.shape[0]):
            obs_dis_x, obs_dis_y = obs[obs_xy][0]-x, obs[obs_xy][1]-y
            obs_dis = np.linalg.norm([obs_dis_x, obs_dis_y])

            if obs_dis < self.obstacle_bound:
                rep_x = rep_x - self.Kp_rel*(1/obs_dis - 1/self.obstacle_bound)*(1/(obs_dis*obs_dis))*obs_dis_x/obs_dis
                rep_y = rep_y - self.Kp_rel*(1/obs_dis - 1/self.obstacle_bound)*(1/(obs_dis*obs_dis))*obs_dis_y/obs_dis
            else:
                rep_x = rep_x
                rep_y = rep_y
            bound = self.make_line_marker(obs[obs_xy], self.obstacle_bound)
            self.obstacle_bound_pub.publish(bound)
        return rep_x, rep_y
                

    def image_cb(self, rgb_image, depth_image):
        bridge = CvBridge()
        rgb_frame = bridge.imgmsg_to_cv2(rgb_image, desired_encoding='rgb8')
        depth_frame = bridge.imgmsg_to_cv2(depth_image, desired_encoding='16UC1')
        try:
            if self.mission == 3:
                start = time.time()
                resolution = (rgb_frame.shape[0], rgb_frame.shape[1])
                results = self.model(cv2.resize(rgb_frame, (640, 640)))
                xyxy = None # Initialize xyx with None
                obstacle = np.array([[self.centroid[0], self.centroid[1], self.centroid[2]]])

                if len(self.crosspos_list) < self.yolo_search_count:
                    error_yaw = math.atan2(self.centroid[1] - self.current_pose.pose.position.y, self.centroid[0]- self.current_pose.pose.position.x)
                    current_angle = error_yaw + math.pi
                    qz = math.sin(error_yaw/2.0)
                    qw = math.cos(error_yaw/2.0)

                    if len(self.crosspos_list) == self.yolo_search_count-5:
                        self.circular_speed *= -1
                    self.target_pose.pose.position.x = self.centroid[0] + self.radius*math.cos(current_angle + self.circular_speed)
                    self.target_pose.pose.position.y = self.centroid[1] + self.radius*math.sin(current_angle + self.circular_speed)
                    self.target_pose.pose.position.z = self.centroid[2]
                    self.target_pose.pose.orientation.x = 0
                    self.target_pose.pose.orientation.y = 0
                    self.target_pose.pose.orientation.z = qz
                    self.target_pose.pose.orientation.w = qw

                elif self.yolo_search_count<=len(self.crosspos_list)<=self.yolo_search_count+self.yolo_stack_count:
                    error_yaw = math.atan2(self.centroid[1] - self.current_pose.pose.position.y, self.centroid[0]- self.current_pose.pose.position.x)
                    current_angle = error_yaw + math.pi
                    qz = math.sin(error_yaw/2.0)
                    qw = math.cos(error_yaw/2.0)
                    self.target_pose.pose.position.x = self.centroid[0] + self.radius*math.cos(current_angle)
                    self.target_pose.pose.position.y = self.centroid[1] + self.radius*math.sin(current_angle)
                    self.target_pose.pose.position.z = self.centroid[2]
                    self.target_pose.pose.orientation.x = 0
                    self.target_pose.pose.orientation.y = 0
                    self.target_pose.pose.orientation.z = qz
                    self.target_pose.pose.orientation.w = qw
                else:
                    setpoint = np.mean(np.array(self.setpoint_list)[16: , :], axis=0)
                    #visualize
                    marker = self.make_cube_marker(setpoint, (0.0, 0.0, 1.0), 0.4)
                    self.approch_pub.publish(marker)

                    goal_x = setpoint[0]#self.current_pose.pose.position.x + (setpoint[0] - self.current_pose.pose.position.x)*0.05
                    goal_y = setpoint[1]#self.current_pose.pose.position.y + (setpoint[1] - self.current_pose.pose.position.y)*0.05

                    att_x, att_y = self.calc_attractive_force(self.current_pose.pose.position.x, self.current_pose.pose.position.y, goal_x, goal_y)
                    rep_x, rep_y = self.calc_repulsive_force(self.current_pose.pose.position.x, self.current_pose.pose.position.y, obstacle)
                    # rospy.loginfo(f"rep_x, rep_y: {rep_x}, {rep_y}")

                    pot_x = att_x + rep_x
                    pot_y = att_y + rep_y

                    target_x = self.current_pose.pose.position.x + pot_x
                    target_y = self.current_pose.pose.position.y + pot_y

                    setpoint[0] = target_x#self.current_pose.pose.position.x + (setpoint[0] - self.current_pose.pose.position.x)*0.05
                    setpoint[1] = target_y#self.current_pose.pose.position.y + (setpoint[1] - self.current_pose.pose.position.y)*0.05
                    setpoint[2] = self.current_pose.pose.position.z + (setpoint[2] - self.current_pose.pose.position.z)*0.2
                    # rospy.loginfo(f"Mean setpoint: {setpoint}")
                    self.target_pose.pose.position.x = setpoint[0]
                    self.target_pose.pose.position.y = setpoint[1]
                    self.target_pose.pose.position.z = setpoint[2]        

                    cross_pos_3d = np.mean(np.array(self.crosspos_list), axis=0)
                    #visualize
                    marker = self.make_cube_marker(cross_pos_3d, (0.0, 0.5, 0.5), 0.4)
                    self.cross_marker_pub.publish(marker)
                    
                    # yaw 계산
                    error_yaw = math.atan2(cross_pos_3d[1] - self.current_pose.pose.position.y, cross_pos_3d[0] - self.current_pose.pose.position.x)
                    qz = math.sin(error_yaw/2.0)
                    qw = math.cos(error_yaw/2.0)
                    self.target_pose.pose.orientation.x = 0.0
                    self.target_pose.pose.orientation.y = 0.0
                    self.target_pose.pose.orientation.z = qz
                    self.target_pose.pose.orientation.w = qw


                for volume in results.xyxy[0]:
                    xyxy = volume.numpy()
                    if xyxy[4] < 0.5:
                        break
                    #resize
                    xyxy[0] = xyxy[0] / 640 * resolution[1]
                    xyxy[2] = xyxy[2] / 640 * resolution[1]
                    xyxy[1] = xyxy[1] / 640 * resolution[0]
                    xyxy[3] = xyxy[3] / 640 * resolution[0]

                    #Center of cross marker in pixel coord
                    cross_pos = ((xyxy[0] + xyxy[2])/2, (xyxy[1] + xyxy[3])/2)
                    #cross makrer sampling in pixel coord
                    other_pos = self.square_sampling((int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])))
                    #tf to 3d
                    cross_pos_3d, _ = self.get_3d_coord_fast([cross_pos], depth_frame)
                    cross_pos_3d = list(cross_pos_3d)[0]
                    self.crosspos_list.append(cross_pos_3d)
                    other_pos_3d, other_depth_mean = self.get_3d_coord_fast(other_pos, depth_frame)

                    # drone position in 3D
                    drone_pos_3d = np.array([self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z])

                    self.dronepos_list.append(drone_pos_3d)

                    #setpoint 계산
                    setpoint = self.cal_approch_setpoint(cross_pos_3d, other_pos_3d, drone_pos_3d, offset=self.offset, depth_mean=other_depth_mean)
                    self.setpoint_pub.publish(self.make_cube_marker(setpoint, (0.5, 0, 0.5), 0.4))

                    if np.isnan(setpoint[0]):
                        rospy.loginfo("NaN Setpoint")
                    else:
                        self.setpoint_list.append(setpoint)
                        self.log_matrices(cross_pos_3d, other_pos_3d, drone_pos_3d, setpoint, id=self.id)
                        self.id += 1


                    if len(self.setpoint_list) >= self.yolo_search_count:
                        setpoint_distance = np.linalg.norm(drone_pos_3d - np.mean(np.array(self.setpoint_list)[self.yolo_search_count: , :], axis=0))
                        cv2.putText(rgb_frame, f'setpoint distance : {setpoint_distance:.2f} : {self.counter}/{self.setpoint_count}', (0, 75), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 128, 0), thickness=2)
                        if setpoint_distance < self.setpoint_criterion:
                            self.counter += 1
                        if self.counter > self.setpoint_count:
                            auto_service.call_drone_command(4)
                        
                    
                    cv2.putText(rgb_frame, f'inference time : {time.time() - start:.3f}', (0, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2)
                    cv2.rectangle(rgb_frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color=(0, 255, 0), thickness=2)
                    cv2.putText(rgb_frame, f'{xyxy[4]:.3f}', (int(xyxy[0]), int(xyxy[1])), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 255, 0), thickness=2)
                    #rospy.loginfo(f"setpoint: {setpoint}")
                    #rospy.loginfo(f"get_2d: {self.get_2d_coord(setpoint)}")
                    try:
                        cv2.line(rgb_frame, (int(cross_pos[0]), int(cross_pos[1])), self.get_2d_coord(cross_pos + (setpoint-cross_pos)*0.5), (255, 0, 0), thickness=3)
                    except:
                        pass
                    # crossmarker break()
                    break

                
                self.target_pose_pub.publish(self.target_pose)
                #rospy.loginfo(f"target pose: {self.target_pose}")
            try:
                cv2.putText(rgb_frame, f"Mission : {int(self.mission)} \"{self.mission_rep}\"", (0, 25), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 0), thickness=2)
                if self.downward is not None:
                    size = 430 if self.mission == 6 else 250
                    downward = cv2.resize(self.downward, (size, size))
                    rgb_frame[-(size+5):, -(size+5):] = np.zeros((size+5, size+5, 3))
                    rgb_frame[-size:, -size:] = downward
                self.image_pub.publish(bridge.cv2_to_imgmsg(rgb_frame, "rgb8"))
            except CvBridgeError as e:
                print(e)


        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node('yolo_tf_node', anonymous=True)
    try:
            image_subscriber = MarkerDetection()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass 