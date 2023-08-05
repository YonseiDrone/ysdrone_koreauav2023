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
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Float32

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


class KalmanFilter(object):
    def __init__(self, dt, u_x, u_y, u_z, std_acc, x_std_meas, y_std_meas, z_std_meas):
        """
        dt : sampling time(time for 1 cycle)
        u_x : acceleration in x-direction
        u_y : acceleration in y-direction
        u_z : acceleration in z-direction
        std_acc : process noise magnitude
        x_std_meas : standard deviation of the measurement in x-direction
        y_std_meas : standard deviation of the measurement in y-direction
        z_std_meas : standard deviation of the measurement in z-direction
        """
        # Define sampling time
        self.dt = dt

        # Define the control input variable
        self.u = np.array([[u_x], [u_y], [u_z]])

        # Initialize the state(x,y,z,x',y',z')
        self.x = np.array([[0], [0], [0], [0], [0], [0]])

        # Define the State Transition Matrix A
        self.A = np.array([[1, 0, 0, self.dt, 0, 0],
                            [0, 1, 0, 0, self.dt, 0],
                            [0, 0, 1, 0, 0, self.dt],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])
        # Define the Control Input Matrix B
        self.B = np.array([[(self.dt**2)/2, 0, 0],
                            [0, (self.dt**2)/2, 0],
                            [0, 0, (self.dt**2)/2],
                            [self.dt, 0, 0],
                            [0, self.dt, 0],
                            [0, 0, self.dt]])
        # Define the Measurement Mapping Matrix
        # Assume that we are only measuring the position but not the velocity
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])
        # Initialize the Process Noise Covariance
        self.Q = np.array([[(self.dt**4)/4, 0, 0, (self.dt**3)/2, 0, 0],
                            [0, (self.dt**4)/4, 0, 0, (self.dt**3)/2, 0],
                            [0, 0, (self.dt**4)/4, 0, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, 0, self.dt**2, 0, 0],
                            [0, (self.dt**3)/2, 0, 0, self.dt**2, 0],
                            [0, 0, (self.dt**3)/2, 0, 0, self.dt**2]]) * (std_acc**2)
        # Initialize the Measurement Noise Covariance
        self.R = np.array([[x_std_meas**2, 0, 0],
                            [0, y_std_meas**2, 0],
                            [0, 0, z_std_meas**2]])
        # Initialize the Covariance Matrix
        # identity array whose shape is the same as the shape of the matrix A
        self.P = np.eye(self.A.shape[1])

    def predict(self):
        # update time state
        # x_k = A*x_(k-1) + B*u_(k-1)
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        # calculate the error covariance
        # P = A*P*A^T + Q
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x[0:3] # return x,y,z
    
    def update(self, z):
        # S = H*P*H^T + R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # calculate the Kalman Gain
        # K = P * H^T * inv(H*P*H^T + R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))), 3)
        I = np.eye(self.H.shape[1])

        # update error covariance matrix
        self.P = (I - (K@self.H)) @ self.P
        return self.x[0:3]
    

class MarkerDetection(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.yoloPath = self.rospack.get_path('yolo_cross_detection') + '/yolov5'
        self.weightPath = self.rospack.get_path('yolo_cross_detection') + '/weight/yolov5nV4.onnx'
        self.model = torch.hub.load(self.yoloPath, 'custom', self.weightPath, source='local', force_reload=True)
        self.model.iou = 0.5
        self.mission = 0
        self.setpoint_list = []
        self.crosspos_list = []
        self.count = 0

        # KalmanFilter(dt, u_x, u_y, u_z, std_acc, x_std_meas, y_std_meas, z_std_meas)
        self.KF = KalmanFilter(0.1, 0.1, 0.1, 0.1, 1, 0.1, 0.1, 0.1)

        #setpoint
        self.offset = 3 # Distance from the cross marker
        self.min_offset = 0.5 # Minimum distance 
        self.offset_interval = 0.1
        self.displacement_steady = 0.2
        self.launch_ready = False

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

    def get_3d_coord(self, pixels, depth_frame):
        depth_frame = depth_frame
        final_coords = []

        for pixel in pixels:
            pixel_x, pixel_y = pixel

            distance = depth_frame[int(pixel_y), int(pixel_x)] # (y, x)
            pixel_center = np.array([int(pixel_x), int(pixel_y)]) # (x, y)
            #rospy.loginfo(f"distance: {distance}")
            #===============================Camera coordinate==========================================
            intrinsic_matrix = np.array([[454.6857718666893, 0.0, 424.5],
                        [0.0, 454.6857718666893, 240.5],
                        [0.0, 0.0, 1.0]])
            camera_center = np.zeros(3)
            camera_center[2] = distance #* 0.001 #z
            camera_center[0] = (pixel_center[0] - intrinsic_matrix[0,2]) * camera_center[2] / intrinsic_matrix[0, 0] #x
            camera_center[1] = (pixel_center[1] - intrinsic_matrix[1,2]) * camera_center[2] / intrinsic_matrix[1, 1] #y
            camera_coord = np.array([camera_center[0], camera_center[1], camera_center[2], 1])
            # rospy.loginfo(f"camera_coord: {camera_coord}")
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
            # rospy.loginfo(f"flu_coord: {flu_coord}")
            #==================================ENU coordinate(east-north-up)=======================================
            _, _, yaw = to_euler_angles(self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w)
            #rospy.loginfo(f"yaw: {yaw*180/math.pi}")

            enu_rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
            enu_translation = np.array([0, 0, 0])

            flu_to_enu = np.eye(4)
            flu_to_enu[:3, :3] = enu_rotation
            flu_to_enu[:3, 3] = enu_translation

            enu_coord = np.dot(flu_to_enu, flu_coord)
            # rospy.loginfo(f"enu_coord: {enu_coord}")
            #=============================Local coordinate(home position)===============
            final_coord_x = self.current_pose.pose.position.x + enu_coord[0]
            final_coord_y = self.current_pose.pose.position.y + enu_coord[1]
            final_coord_z = self.current_pose.pose.position.z + enu_coord[2]

            final_coords.append([final_coord_x, final_coord_y, final_coord_z])

            #rospy.loginfo(f"Drone current position: {self.current_pose.pose.position.x}, {self.current_pose.pose.position.y}, {self.current_pose.pose.position.z}")
            #rospy.loginfo(f"Cross Marker position: {final_coord_x}, {final_coord_y}, {final_coord_z}")
            self.final_coord.pose.position.x = final_coord_x
            self.final_coord.pose.position.y = final_coord_y
            self.final_coord.pose.position.z = final_coord_z
            self.final_coord_pub.publish(self.final_coord)


            #======================FAKE==========================================
            # self.KF.predict()
            # z = np.array([[enu_coord[0]], [enu_coord[1]], [enu_coord[2]]])
            # enu_coord[0:3] =self.KF.update(z).flatten()
            # rospy.loginfo(f"enu_coord: {enu_coord[0:3]}")
            #final_coords.append(enu_coord[0:3])
            #====================================================================

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
        # _, _, yaw = to_euler_angles(self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w)
        _, _, yaw = to_euler_angles(self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w)
        yaw *= -1
        enu_rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        enu_translation = np.array([0, 0, 0])

        enu_to_flu = np.eye(4)
        enu_to_flu[:3, :3] = enu_rotation
        enu_to_flu[:3, 3] = enu_translation

        flu_coord = np.dot(enu_to_flu, enu_coord)

        #=====================FAKE=====================
        #flu_coord = np.dot(enu_to_flu, position)
        #==============================================


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
        #rospy.loginfo(f"image_coordinate: {image_coordinates}")

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

                resolution = (rgb_frame.shape[0], rgb_frame.shape[1])
                results = self.model(cv2.resize(rgb_frame, (640, 640)))
                xyxy = None # Initialize xyx with None

                self.count += 1
                if len(self.crosspos_list) < 100:
                    self.target_pose.pose.position.x = self.current_pose.pose.position.x
                    self.target_pose.pose.position.y = self.current_pose.pose.position.y
                    self.target_pose.pose.position.z = self.current_pose.pose.position.z

                    
                if self.count == 100:
                    rospy.loginfo("==========================HERE==================================")
                    setpoint = np.mean(np.array(self.setpoint_list), axis=0)
                    rospy.loginfo(f"Mean setpoint: {setpoint}")
                    self.target_pose.pose.position.x = setpoint[0]
                    self.target_pose.pose.position.y = setpoint[1]
                    self.target_pose.pose.position.z = setpoint[2]

                    cross_pos_3d = np.mean(np.array(self.crosspos_list), axis=0)
                    rospy.loginfo(f"Mean cross marekr: {cross_pos_3d}")
                    
                    # yaw 계산
                    error_yaw = math.atan2(cross_pos_3d[1] - self.target_pose.pose.position.y, cross_pos_3d[0] - self.target_pose.pose.position.x)
                    qz = math.sin(error_yaw/2.0)
                    qw = math.cos(error_yaw/2.0)
                    self.target_pose.pose.orientation.x = 0.0
                    self.target_pose.pose.orientation.y = 0.0
                    self.target_pose.pose.orientation.z = qz
                    self.target_pose.pose.orientation.w = qw

                    #self.count = 0
                    self.setpoint_list = []

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
                    cross_pos_3d = self.get_3d_coord([cross_pos], depth_frame)[0]
                    self.crosspos_list.append(cross_pos_3d)
                    #rospy.loginfo(f"Cross Marker Position: {cross_pos_3d}")
                    other_pos_3d = self.get_3d_coord(other_pos, depth_frame)
                    # rospy.loginfo(f"other_pos_3d {other_pos_3d}")

                    # drone position in 3D
                    drone_pos_3d = np.array([self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z])
                    #rospy.loginfo(f"Current Drone Position: {drone_pos_3d}")

                    #============================FAKE================================
                    #drone_pos_3d = np.array([0.0, 0.0, 0.0])
                    #===============================================================

                    #setpoint 계산
                    setpoint = self.cal_approch_setpoint(cross_pos_3d, other_pos_3d, drone_pos_3d, offset=self.offset)
                    self.setpoint_list.append(setpoint)  

                    #rospy.loginfo(f"target position: {self.target_pose}")
                    #rospy.loginfo(f"Veranda Approch SetPoint : {setpoint}")
                    cv2.rectangle(rgb_frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color=(0, 255, 0), thickness=2)
                    cv2.putText(rgb_frame, f'{xyxy[4]:.3f}', (int(xyxy[0]), int(xyxy[1])), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 255, 0), thickness=2)
                    cv2.line(rgb_frame, (int(cross_pos[0]), int(cross_pos[1])), self.get_2d_coord(setpoint), (255, 0, 0), thickness=3)
                    # crossmarker break
                    break
    
                

                # Publish setpoint
                # self.KF.predict()
                # z = np.array([[setpoint[0]], [setpoint[1]], [setpoint[2]]])
                # setpoint[0:3] =self.KF.update(z).flatten()

                
                self.target_pose_pub.publish(self.target_pose)
                rospy.loginfo(f"target pose: {self.target_pose}")
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