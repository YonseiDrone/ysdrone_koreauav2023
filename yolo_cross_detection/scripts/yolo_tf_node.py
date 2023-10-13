#!/usr/bin/env python3

import rospy, rospkg

# ML, Calculation pkgs
import cv2, torch
import numpy as np
from sklearn.decomposition import PCA
from scipy.stats import zscore
import math, time, os, datetime

from cv_bridge import CvBridge, CvBridgeError

# import msgs...
import message_filters
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from mavros_msgs.msg import State
from std_msgs.msg import Float32, String

# custom pkgs
from ysdrone_msgs.srv import *
from koreauav_utils import auto_service
from yolo_utils import *

class MarkerDetection(object):
    def __init__(self):
        self.last_detection_time = time.time()
        self.rospack = rospkg.RosPack()

        # load yolov5 model from local
        self.yoloPath = self.rospack.get_path('yolo_cross_detection') + '/yolov5'
        self.weightPath = self.rospack.get_path('yolo_cross_detection') + '/weight/yolov5nV4.onnx'
        self.model = torch.hub.load(self.yoloPath, 'custom', self.weightPath, source='local', force_reload=True)
        self.model.iou = 0.5

        # mission info
        self.mission = 0
        self.mission_rep = ""

        # list to stack setpoint, crossmarker point
        self.setpoint_list = []
        self.crosspos_list = []

        # setpoint approach counter
        self.counter = 0

        # used for log matrices
        self.id = 0

        # PCA feature extracotr
        self.pca = PCA(n_components=2)

        # Realsense Intrinsic
        self.intrinsic_matrix = np.array([[385.7627868652344, 0.0, 331.9479064941406],
                        [0.0, 385.4613342285156, 237.6436767578125],
                        [0.0, 0.0, 1.0]])
        
        # Parameter for Potential Field
        self.Kp_att = 0.1
        self.Kp_rel = 2500.0

        # ROSPARM get_param()
        self.radius = rospy.get_param("radius", 6) # 원주 비행 시 건물 중심으로부터의 반지름
        self.obstacle_bound = rospy.get_param("obstacle_bound",2) # 건물 중심으로부터 Potential Field가 작동되는 반지름 범위
        
        self.circular_speed = rospy.get_param("circular_speed", 0.1) # 원주 비행 시 각속도
        self.yolo_search_count = rospy.get_param("yolo_search_count", 7) # 마커를 찾았다고 판단할 때 사용하는 crosspos_list의 개수
        self.yolo_stack_count = rospy.get_param("yolo_stack_count", 15) # 마커의 값을 평균낼 때 사용하는 crosspos_list의 개수
        self.setpoint_count = rospy.get_param("setpoint_count",10) # setpoint에 도착했다고 판단할 때 사용하는 self.count의 수

        self.offset = rospy.get_param("yolo_offset", 5) # 마커로부터 setpoint의 거리
        self.setpoint_criterion = rospy.get_param("setpoint_criterion", 0.3) # setpoint에 도착했다고 판단하기 위한 기준(m)

        # initiate classes for callback
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.final_coord = PoseStamped()
        self.imu = Imu()
        self.target_pose = PoseStamped()
        self.centroid = None
        self.downward = None

        # Subscriber
        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw', Image) # front cam rgb
        self.depth_image_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image) # front cam depth
        self.downward_sub = rospy.Subscriber('/cv_image', Image, self.downward_cb) # downward cam

        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb) # drone states
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)

        self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb) # mission number
        self.mission_rep_sub = rospy.Subscriber('/mission_rep', String, self.mission_rep_cb) # mission name

        self.centroid_sub = rospy.Subscriber('/building/search/centroid_pose', PoseStamped, self.centroid_cb) # building centroid from mission 2

        # Synchronize the topics
        ts = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.image_cb)

        # Publisher
        self.image_pub = rospy.Publisher('/cross_image', Image, queue_size=1) # front cam image for broadcast
        self.target_pose_pub = rospy.Publisher('/launch_setposition', PoseStamped, queue_size=1) # target position
        
        # Gazebo Marker Publishers
        self.centroid_pub = rospy.Publisher('/building_centroid', Marker, queue_size=1)
        self.obstacle_bound_pub = rospy.Publisher('/obstacle_bound', Marker, queue_size=1)
        self.approch_pub = rospy.Publisher('/cross_marker_approch_setpoint', Marker, queue_size=1)
        self.cross_marker_pub = rospy.Publisher('/cross_marker_position', Marker, queue_size=1)
        
        # Make Directory for Matrix log
        self.logd = f"/home/khadas/logs_{datetime.datetime.now().strftime('%Y%m%d-%H%M%S')}"
        rospy.loginfo(f'npy log dir : {self.logd}')
        os.mkdir(self.logd)

    # ================================Subscriber Callbacks===========================================
    def centroid_cb(self, msg): # store centroid, publish to gazebo
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        self.centroid = np.array([x,y,z])
        marker = make_cube_marker(self.centroid, (0, 255, 0), 0.6)
        self.centroid_pub.publish(marker)
    
    def mission_cb(self, msg): # store mission number
        self.mission = msg.data

    def mission_rep_cb(self, msg): # store mission name
        self.mission_rep = msg.data

    def imu_cb(self, msg): # store drone imu
        self.imu = msg

    def downward_cb(self, msg): # store downward cam img
        bridge = CvBridge()
        self.downward = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def state_cb(self, msg): # store drone state
        prev_state = self.current_state
        self.current_state = msg

        if self.current_state.mode != prev_state.mode:
            rospy.loginfo(f"Current Mode : {self.current_state.mode}")
        if self.current_state.armed != prev_state.armed:
            rospy.loginfo(f"Vehicle armed : {self.current_state.armed}")
    
    def pose_cb(self, msg): # store drone pose
        self.current_pose = msg

    # ================================Setpoint Calculation Functions========================================================================
    def get_3d_coord_fast(self, pixels, depth_frame): # Pixel -> Camera -> FLU -> ENU -> Local
        pixels = np.array(pixels).astype(np.int32)
        distances = depth_frame[pixels[:, 1], pixels[:, 0]]
        # Remove NaN values from numpy array 
        depth_mean = np.mean(distances[~np.isnan(distances)]) * 0.001 # unit convert (mm to m)

        # Pixel to Camera
        camera_coords = np.ones((4, len(distances)))
        camera_coords[2, :] = distances*0.001
        camera_coords[0, :] = (pixels[:, 0] - self.intrinsic_matrix[0, 2]) * camera_coords[2, :] / self.intrinsic_matrix[0, 0] #x
        camera_coords[1, :] = (pixels[:, 1] - self.intrinsic_matrix[1, 2]) * camera_coords[2, :] / self.intrinsic_matrix[1, 1] #y

        # Camera to FLU
        camera_to_flu = np.array([[ 6.12323400e-17,  0.00000000e+00,  1.00000000e+00,  0.00000000e+00], 
                                [-1.00000000e+00,  6.12323400e-17,  6.12323400e-17,  0.00000000e+00], 
                                [-6.12323400e-17, -1.00000000e+00,  3.74939946e-33,  0.00000000e+00], 
                                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        flu_coords = np.dot(camera_to_flu, camera_coords)
        
        # FLU to ENU
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

        # ENU to Local position
        enu_coords[0, :] += self.current_pose.pose.position.x
        enu_coords[1, :] += self.current_pose.pose.position.y
        enu_coords[2, :] += self.current_pose.pose.position.z
        enu_coords = enu_coords.T
    
        return enu_coords[:,0:3], depth_mean
    
    # inverse tf of get_3d_coord
    def get_2d_coord(self, position): # Local -> ENU -> FLU -> Camera -> Pixel
        # Local to ENU coordinate(drone position)
        position = np.array([position[0], position[1], position[2], 1])
        enu_coord = np.ones(4)
        enu_coord[0] = position[0] - self.current_pose.pose.position.x
        enu_coord[1] = position[1] - self.current_pose.pose.position.y
        enu_coord[2] = position[2] - self.current_pose.pose.position.z

        # ENU to FLU coordinate(east-north-up)
        _, _, yaw = to_euler_angles(self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w)
        yaw *= -1
        enu_rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        enu_translation = np.array([0, 0, 0])
        enu_to_flu = np.eye(4)
        enu_to_flu[:3, :3] = enu_rotation
        enu_to_flu[:3, 3] = enu_translation
        flu_coord = np.dot(enu_to_flu, enu_coord)

        # FLU to Camera coordinate(front-left-up)
        x_rotation = -90 * math.pi / 180
        y_rotation = 90 * math.pi / 180
        z_rotation = 0 
        flu_x_rotation = np.array([[1,0,0], [0, np.cos(x_rotation), -np.sin(x_rotation)], [0, np.sin(x_rotation), np.cos(x_rotation)]])
        flu_y_rotation = np.array([[np.cos(y_rotation),0,np.sin(y_rotation)], [0,1,0], [-np.sin(y_rotation), 0, np.cos(y_rotation)]])
        flu_z_rotation = np.array([[np.cos(z_rotation), -np.sin(z_rotation), 0], [np.sin(z_rotation), np.cos(z_rotation), 0], [0,0,1]])
        flu_rotation = np.dot(flu_x_rotation, flu_y_rotation)
        flu_rotation = np.linalg.inv(flu_rotation)
        flu_translation = np.array([0, 0, 0])
        flu_to_cam = np.eye(4)
        flu_to_cam[:3, :3] = flu_rotation
        flu_to_cam[:3, 3] = flu_translation
        cam_coord = np.dot(flu_to_cam, flu_coord)

        # Camera to Pixel coordinate
        cam_coord = cam_coord[:3]
        image_coordinates = np.dot(self.intrinsic_matrix, cam_coord)
        u = image_coordinates[0] / image_coordinates[2]
        v = image_coordinates[1] / image_coordinates[2]

        return [int(u), int(v)]
    
    def cal_approch_setpoint(self, cross_pos, other_pos, drone_pos, offset, depth_mean): # calculate setpoint
        points = np.array(other_pos) # to numpy array

        # remove z-score >= 2.5 points
        z_scores = np.abs(zscore(points[:, :2]))
        z_scores = np.sqrt(z_scores[:, 0] ** 2 + z_scores[:, 1] ** 2)
        inliers = z_scores < 2.5

        # Projection to xy plane
        points = points[inliers, :2]

        if points.shape[0] < 3: # samples not reliable
            nan_array = np.array([np.nan, np.nan, np.nan])
            return nan_array
        else:
            # use pca to find principle components
            param = self.pca.fit(points).components_
            normal_vector = np.array([param[-1][0], param[-1][1], 0]) # second principle is normal vector
            cross_drone_vec = cross_pos - drone_pos

            if np.dot(normal_vector, cross_drone_vec) > 0: # To make the normal vector point towards the drone
                normal_vector = -normal_vector

            # drone setpoint = cross marker pos + offset * normal vector
            return cross_pos + normal_vector / np.linalg.norm(normal_vector) * offset * offset / (depth_mean if depth_mean < offset else offset)
#=======================================================================================================================================================


    def image_cb(self, rgb_image, depth_image): # main callback
        bridge = CvBridge()
        rgb_frame = bridge.imgmsg_to_cv2(rgb_image, desired_encoding='rgb8')
        depth_frame = bridge.imgmsg_to_cv2(depth_image, desired_encoding='16UC1')
        try:
            if self.mission == 3: # yolo mode
                start = time.time() # inference start time
                resolution = (rgb_frame.shape[0], rgb_frame.shape[1]) # resolution of img
                results = self.model(cv2.resize(rgb_frame, (640, 640))) # run yolov5 model
                xyxy = None # Initialize xyx with None
                obstacle = np.array([[self.centroid[0], self.centroid[1], self.centroid[2]]]) # obstacle centroid

                if len(self.crosspos_list) < self.yolo_search_count: # crossmarker not found, circular flight based on building centroid
                    error_yaw = math.atan2(self.centroid[1] - self.current_pose.pose.position.y, self.centroid[0] - self.current_pose.pose.position.x)
                    current_angle = error_yaw + math.pi
                    qz = math.sin(error_yaw/2.0)
                    qw = math.cos(error_yaw/2.0)
                    self.target_pose.pose.position.x = self.current_pose.pose.position.x + (self.centroid[0] + self.radius*math.cos(current_angle + self.circular_speed) - self.current_pose.pose.position.x)*0.4
                    self.target_pose.pose.position.y = self.current_pose.pose.position.y + (self.centroid[1] + self.radius*math.sin(current_angle + self.circular_speed) - self.current_pose.pose.position.y)*0.4
                    self.target_pose.pose.position.z = self.centroid[2]
                    self.target_pose.pose.orientation.x = 0
                    self.target_pose.pose.orientation.y = 0
                    self.target_pose.pose.orientation.z = qz
                    self.target_pose.pose.orientation.w = qw
                    self.target_pose_pub.publish(self.target_pose)
                
                elif self.yolo_search_count<=len(self.crosspos_list)<=self.yolo_search_count+self.yolo_stack_count: # marker found, stop and stack datas
                    error_yaw = math.atan2(self.centroid[1] - self.current_pose.pose.position.y, self.centroid[0] - self.current_pose.pose.position.x)
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
                    self.target_pose_pub.publish(self.target_pose)
                    
                    if time.time() - self.last_detection_time > 3: # marker dissapeared, retry circular flight
                        self.crosspos_list = []
                        self.setpoint_list = []

                else: # setpoint calculated, approach to setpoint
                    setpoint = np.mean(np.array(self.setpoint_list)[7: , :], axis=0)
                    marker = make_cube_marker(setpoint, (0.0, 0.0, 1.0), 0.4)
                    self.approch_pub.publish(marker)

                    goal_x = setpoint[0]
                    goal_y = setpoint[1]

                    att_x, att_y = calc_attractive_force(self.Kp_att, self.current_pose.pose.position.x, self.current_pose.pose.position.y, goal_x, goal_y)
                    rep_x, rep_y = calc_repulsive_force(self.Kp_rel, self.current_pose.pose.position.x, self.current_pose.pose.position.y, obstacle, self.obstacle_bound)

                    bound = make_line_marker(obstacle[0], self.obstacle_bound)
                    self.obstacle_bound_pub.publish(bound)

                    pot_x = att_x + rep_x
                    pot_y = att_y + rep_y

                    target_x = self.current_pose.pose.position.x + pot_x
                    target_y = self.current_pose.pose.position.y + pot_y

                    setpoint[0] = target_x
                    setpoint[1] = target_y
                    setpoint[2] = self.current_pose.pose.position.z + (setpoint[2] - self.current_pose.pose.position.z) * 0.5

                    self.target_pose.pose.position.x = setpoint[0]
                    self.target_pose.pose.position.y = setpoint[1]
                    self.target_pose.pose.position.z = setpoint[2]

                    cross_pos_3d = np.mean(np.array(self.crosspos_list), axis=0)
                    marker = make_cube_marker(cross_pos_3d, (0.0, 0.0, 1.0), 0.4)
                    self.cross_marker_pub.publish(marker)
                    
                    # yaw 계산
                    error_yaw = math.atan2(cross_pos_3d[1] - self.target_pose.pose.position.y, cross_pos_3d[0] - self.target_pose.pose.position.x)
                    qz = math.sin(error_yaw/2.0)
                    qw = math.cos(error_yaw/2.0)
                    self.target_pose.pose.orientation.x = 0.0
                    self.target_pose.pose.orientation.y = 0.0
                    self.target_pose.pose.orientation.z = qz
                    self.target_pose.pose.orientation.w = qw
                    self.target_pose_pub.publish(self.target_pose)

                # Analyze yolo, stack results, calculate setpoints
                for volume in results.xyxy[0]:
                    xyxy = volume.numpy()
                    if xyxy[4] < 0.5: # not confident
                        break
                    
                    self.last_detection_time = time.time()
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

                    #setpoint 계산
                    if other_pos_3d.shape[0] > 10:
                        setpoint = self.cal_approch_setpoint(cross_pos_3d, other_pos_3d, drone_pos_3d, offset=self.offset, depth_mean=other_depth_mean)
                    else:
                        break
                    # 신의 한 수
                    if np.isnan(setpoint[0]):
                        rospy.loginfo("NaN Setpoint")
                    else:
                        self.setpoint_list.append(setpoint)
                        self.log_npys(logd=self.logd, cross_pos_3d=cross_pos_3d, other_pos_3d=other_pos_3d, drone_pos_3d=drone_pos_3d, setpoint=setpoint, id=self.id)
                        self.id += 1

                    # visualize setpoint information & counter check
                    if len(self.setpoint_list) >= self.yolo_search_count:
                        setpoint_distance = np.linalg.norm(drone_pos_3d - np.mean(np.array(self.setpoint_list)[self.yolo_search_count: , :], axis=0))

                        cv2.putText(rgb_frame, f'setpoint distance : {setpoint_distance:.2f} : {self.counter}/{self.setpoint_count}', (0, 75), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 128, 0), thickness=2)

                        if setpoint_distance < self.setpoint_criterion: # approached, counter up
                            self.counter += 1
                        if self.counter > self.setpoint_count: # finally approached, move to Cargo Launch Mode
                            auto_service.call_drone_command(4)                 

                    # visualize yolo inference information
                    cv2.putText(rgb_frame, f'inference time : {time.time() - start:.3f}', (0, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2)
                    cv2.rectangle(rgb_frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color=(0, 255, 0), thickness=2)
                    cv2.putText(rgb_frame, f'{xyxy[4]:.3f}', (int(xyxy[0]), int(xyxy[1])), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 255, 0), thickness=2)
                    try:
                        cv2.line(rgb_frame, (int(cross_pos[0]), int(cross_pos[1])), self.get_2d_coord(setpoint), (255, 0, 0), thickness=3)
                    except:
                        pass
                    # crossmarker break
                    break

            try:
                # visualize mission information, downward cam (running even not yolo mode)
                cv2.putText(rgb_frame, f"Mission : {int(self.mission)} \"{self.mission_rep}\"", (0, 25), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 0), thickness=2)
                if self.downward is not None:
                    size = 430 if self.mission in [6, 9, 10] else 200 # make larger if mode is landing mode
                    downward = cv2.resize(self.downward, (size, size))
                    rgb_frame[-(size+5):, -(size+5):] = np.zeros((size+5, size+5, 3))
                    rgb_frame[-size:, -size:] = downward
                image_msg = bridge.cv2_to_imgmsg(rgb_frame, "rgb8")
                image_msg.header.stamp = rospy.Time.now()
                self.image_pub.publish(image_msg)
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
