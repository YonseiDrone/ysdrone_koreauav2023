#!/usr/bin/env python3

import rospy, rospkg
import cv2, torch
import numpy as np
from scipy.linalg import svd
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge, CvBridgeError
from ysdrone_msgs.srv import *
from geometry_msgs.msg import Point
import message_filters
import math, time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas


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
        self.setpoint_list = []
        self.crosspos_list = []

        #setpoint
        self.offset = 3 # Distance from the cross marker
        self.circular_speed = 0.2

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.final_coord = PoseStamped()

        self.imu = Imu()
        self.target_pose = PoseStamped()

        # Subscriber
        self.rgb_image_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        self.depth_image_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)

        # Synchronize the topics
        ts = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.image_cb)

        # Publisher
        self.image_pub = rospy.Publisher('/cross_image', Image, queue_size=1)
        self.final_coord_pub = rospy.Publisher('/marker_position/home', PoseStamped, queue_size=1)
        self.target_pose_pub = rospy.Publisher('/launch_setposition', PoseStamped, queue_size=1)

        rospy.on_shutdown(self.visualize)

    
    def visualize(self):
        rospy.loginfo("=================VISUAL================")
        fig = plt.figure()
        ax = fig.add_subplot(111,projection='3d')
        self.setpoint_list = np.array(self.setpoint_list)
        self.crosspos_list = np.array(self.crosspos_list)
        setpoint_x = self.setpoint_list[:, 0]
        setpoint_y = self.setpoint_list[:, 1]
        setpoint_z = self.setpoint_list[:, 2]
        cross_x = self.crosspos_list[:, 0]
        cross_y = self.crosspos_list[:,1]
        cross_z = self.crosspos_list[:,2]
        df = pandas.DataFrame()
        df['setpoint_x'] = setpoint_x
        df['setpoint_y'] = setpoint_y
        df['setpoint_z'] = setpoint_z
        df['cross_x'] = cross_x
        df['cross_y'] = cross_y
        df['cross_z'] = cross_z
        df.to_csv('~/Downloads/3d.csv')


    def mission_cb(self, msg):
        self.mission = msg.data

    def imu_cb(self, msg):
        self.imu = msg

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
        intrinsic_matrix = np.array([[454.6857718666893, 0.0, 424.5],
                        [0.0, 454.6857718666893, 240.5],
                        [0.0, 0.0, 1.0]])
        pixels = np.array(pixels).astype(np.int32)
        #rospy.loginfo(f"pixels shape: {pixels.shape}")
        distances = depth_frame[pixels[:, 1], pixels[:, 0]]
        #rospy.loginfo(f"distances: {distances}")
        #rospy.loginfo(f"distances shapes: {distances.shape}")
        #=====================Image to Camera======================================================================
        camera_coords = np.ones((4, len(distances)))
        camera_coords[2, :] = distances
        camera_coords[0, :] = (pixels[:, 0] - intrinsic_matrix[0, 2]) * camera_coords[2, :] / intrinsic_matrix[0, 0] #x
        camera_coords[1, :] = (pixels[:, 1] - intrinsic_matrix[1, 2]) * camera_coords[2, :] / intrinsic_matrix[1, 1] #y
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
    
        return enu_coords[:,0:3]
    
    def get_3d_coord(self, pixels, depth_frame):
        depth_frame = depth_frame
        final_coords = []

        for pixel in pixels:
            pixel_x, pixel_y = pixel

            distance = depth_frame[int(pixel_y), int(pixel_x)] # (y, x)
            pixel_center = np.array([int(pixel_x), int(pixel_y)]) # (x, y)
            #===============================Camera coordinate==========================================
            intrinsic_matrix = np.array([[454.6857718666893, 0.0, 424.5],
                        [0.0, 454.6857718666893, 240.5],
                        [0.0, 0.0, 1.0]])
            camera_center = np.zeros(3)
            camera_center[2] = distance #z
            camera_center[0] = (pixel_center[0] - intrinsic_matrix[0,2]) * camera_center[2] / intrinsic_matrix[0, 0] #x
            camera_center[1] = (pixel_center[1] - intrinsic_matrix[1,2]) * camera_center[2] / intrinsic_matrix[1, 1] #y
            camera_coord = np.array([camera_center[0], camera_center[1], camera_center[2], 1])
            #===============================FLU coordinate(front-left-up)==========================================
            x_rotation = -90 * math.pi /180
            y_rotation = 90 * math.pi /180
            z_rotation = 0 
            flu_x_rotation = np.array([[1,0,0], [0, np.cos(x_rotation), -np.sin(x_rotation)], [0, np.sin(x_rotation), np.cos(x_rotation)]])
            flu_y_rotation = np.array([[np.cos(y_rotation),0,np.sin(y_rotation)], [0,1,0], [-np.sin(y_rotation), 0, np.cos(y_rotation)]])
            flu_z_rotation = np.array([[np.cos(z_rotation), -np.sin(z_rotation), 0], [np.sin(z_rotation), np.cos(z_rotation), 0], [0,0,1]])
            flu_rotation = np.dot(flu_x_rotation, flu_y_rotation)
            flu_translation = np.array([0, 0, 0])

            camera_to_flu = np.eye(4)
            camera_to_flu[:3, :3] = flu_rotation
            camera_to_flu[:3, 3] = flu_translation

            flu_coord = np.dot(camera_to_flu, camera_coord)
            #==================================ENU coordinate(east-north-up)=======================================
            roll, pitch, yaw = to_euler_angles(self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w)

            enu_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
            enu_roll = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
            enu_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
            enu_rotation = np.dot(enu_yaw, enu_pitch, enu_roll)
            enu_translation = np.array([0, 0, 0])

            flu_to_enu = np.eye(4)
            flu_to_enu[:3, :3] = enu_rotation
            flu_to_enu[:3, 3] = enu_translation

            enu_coord = np.dot(flu_to_enu, flu_coord)
            #=============================Local coordinate(home position)===============
            final_coord_x = self.current_pose.pose.position.x + enu_coord[0]
            final_coord_y = self.current_pose.pose.position.y + enu_coord[1]
            final_coord_z = self.current_pose.pose.position.z + enu_coord[2]

            final_coords.append([final_coord_x, final_coord_y, final_coord_z])

            self.final_coord.pose.position.x = final_coord_x
            self.final_coord.pose.position.y = final_coord_y
            self.final_coord.pose.position.z = final_coord_z
            self.final_coord_pub.publish(self.final_coord)

        return final_coords
    
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
        flu_z_rotation = np.array([[np.cos(z_rotation), -np.sin(z_rotation), 0], [np.sin(z_rotation), np.cos(z_rotation), 0], [0,0,1]])
        flu_rotation = np.dot(flu_x_rotation, flu_y_rotation)
        flu_rotation = np.linalg.inv(flu_rotation)
        flu_translation = np.array([0, 0, 0])

        flu_to_cam = np.eye(4)
        flu_to_cam[:3, :3] = flu_rotation
        flu_to_cam[:3, 3] = flu_translation

        cam_coord = np.dot(flu_to_cam, flu_coord)

        #===============================Camera to Pixel coordinate==========================================
        intrinsic_matrix = np.array([[454.6857718666893, 0.0, 424.5],
                        [0.0, 454.6857718666893, 240.5],
                        [0.0, 0.0, 1.0]])
        cam_coord = cam_coord[:3]
        image_coordinates = np.dot(intrinsic_matrix, cam_coord)
        u = image_coordinates[0] / image_coordinates[2]
        v = image_coordinates[1] / image_coordinates[2]

        return [int(u), int(v)]
    
    
    def square_sampling(self, left_top, right_bottom, interval=5):
        result = []
        x_start, y_start = left_top
        x_end, y_end = right_bottom

        for x in range(x_start, x_end + 1, interval):
            for y in range(y_start, y_end + 1, interval):
                result.append((x, y))

        return result
    
    def cal_approch_setpoint(self, cross_pos, other_pos, drone_pos, offset):
        n = len(other_pos)
        points = np.array(other_pos)

        mean_point = np.mean(points, axis=0)
        centered_points = points - mean_point
        # SVD(Singular Value Decomposition)
        _, _, vh = svd(centered_points)
        normal_vector = vh[-1]
        normal_vector[2] = 0
        
        cross_drone_vec = cross_pos - drone_pos

        if np.dot(normal_vector, cross_drone_vec) > 0:
            normal_vector = -normal_vector

        # drone setpoint
        return cross_pos + normal_vector / np.sqrt(np.sum(normal_vector * normal_vector)) * offset

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


                if len(self.crosspos_list) < 15:
                    radius = 4
                    error_yaw = math.atan2(-42.9 - self.current_pose.pose.position.y, 71.46 - self.current_pose.pose.position.x)
                    current_angle = error_yaw + math.pi
                    qz = math.sin(error_yaw/2.0)
                    qw = math.cos(error_yaw/2.0)
                    self.target_pose.pose.position.x = 71.46 + radius*math.cos(current_angle + self.circular_speed)
                    self.target_pose.pose.position.y = -42.9 + radius*math.sin(current_angle + self.circular_speed)
                    self.target_pose.pose.position.z = 9
                    self.target_pose.pose.orientation.x = 0
                    self.target_pose.pose.orientation.y = 0
                    self.target_pose.pose.orientation.z = qz
                    self.target_pose.pose.orientation.w = qw
                
                elif 15<=len(self.crosspos_list)<=30:
                    radius = 4
                    error_yaw = math.atan2(-42.9 - self.current_pose.pose.position.y, 71.46 - self.current_pose.pose.position.x)
                    current_angle = error_yaw + math.pi
                    qz = math.sin(error_yaw/2.0)
                    qw = math.cos(error_yaw/2.0)
                    self.target_pose.pose.position.x = 71.46 + radius*math.cos(current_angle)
                    self.target_pose.pose.position.y = -42.9 + radius*math.sin(current_angle)
                    self.target_pose.pose.position.z = 9
                    self.target_pose.pose.orientation.x = 0
                    self.target_pose.pose.orientation.y = 0
                    self.target_pose.pose.orientation.z = qz
                    self.target_pose.pose.orientation.w = qw

                    
                else:
    
                    setpoint = np.mean(np.array(self.setpoint_list)[16: , :], axis=0)

                    setpoint[0] = self.current_pose.pose.position.x + (setpoint[0] - self.current_pose.pose.position.x)*0.05
                    setpoint[1] = self.current_pose.pose.position.y + (setpoint[1] - self.current_pose.pose.position.y)*0.05
                    setpoint[2] = self.current_pose.pose.position.z + (setpoint[2] - self.current_pose.pose.position.z)*0.05
                    rospy.loginfo(f"Mean setpoint: {setpoint}")
                    self.target_pose.pose.position.x = setpoint[0]
                    self.target_pose.pose.position.y = setpoint[1]
                    self.target_pose.pose.position.z = setpoint[2]

                    cross_pos_3d = np.mean(np.array(self.crosspos_list), axis=0)
                    rospy.loginfo(f"Mean cross marker: {cross_pos_3d}")
                    
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
                    cross_pos_3d = self.get_3d_coord_fast([cross_pos], depth_frame)[0]
                    self.crosspos_list.append(cross_pos_3d)
                    other_pos_3d = self.get_3d_coord_fast(other_pos, depth_frame)

                    # drone position in 3D
                    drone_pos_3d = np.array([self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z])

                    #setpoint 계산
                    setpoint = self.cal_approch_setpoint(cross_pos_3d, other_pos_3d, drone_pos_3d, offset=self.offset)
                    self.setpoint_list.append(setpoint)  
                    
                    cv2.putText(rgb_frame, f'inference time : {time.time() - start:.3f}', (0, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2)
                    cv2.rectangle(rgb_frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color=(0, 255, 0), thickness=2)
                    cv2.putText(rgb_frame, f'{xyxy[4]:.3f}', (int(xyxy[0]), int(xyxy[1])), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 255, 0), thickness=2)
                    #rospy.loginfo(f"setpoint: {setpoint}")
                    #rospy.loginfo(f"get_2d: {self.get_2d_coord(setpoint)}")
                    #cv2.line(rgb_frame, (int(cross_pos[0]), int(cross_pos[1])), self.get_2d_coord(setpoint), (255, 0, 0), thickness=3)
                    # crossmarker break()
                    break

                
                self.target_pose_pub.publish(self.target_pose)
                #rospy.loginfo(f"target pose: {self.target_pose}")
            try:
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