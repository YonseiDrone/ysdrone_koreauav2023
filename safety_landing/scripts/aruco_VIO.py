#!/usr/bin/env python3
from __future__ import print_function
import time
import roslib
import sys
import math
import re
import os

import rospy
import cv2
import glob
import numpy as np
import cv2.aruco as aruco

from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point

def get_usb_device():
# https://stackoverflow.com/questions/35821763/create-opencv-videocapture-from-interface-name-instead-of-camera-numbers
    DEFAULT_CAMERA_NAME = '/dev/v4l/by-path/platform-xhci-hcd.0.auto-usb-0:1.2:1.0-video-index0'
    device_index = None
    if os.path.exists(DEFAULT_CAMERA_NAME):
        device_path = os.path.realpath(DEFAULT_CAMERA_NAME)
        device_re = re.compile("\/dev\/video(\d+)")
        info = device_re.match(device_path)
        print(f"Checking device_path: {device_path}")
        if info:
            device_index = int(info.group(1))
            print("Using device on /dev/video" + str(device_index))
    
    return device_index

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


class KalmanFilter(object):
    def __init__(self, dt, u_x, u_y, std_acc, x_std_meas, y_std_meas):
        """
        dt : sampling time(time for 1 cycle)
        u_x : acceleration in x-direction
        u_y : acceleration in y-direction
        std_acc : process noise magnitude
        x_std_meas : standard deviation of the measurement in x-direction
        y_std_meas : standard deviation of the measurement in y-direction
        """
        # Define sampling time
        self.dt = dt

        # Define the control input variable
        self.u = np.array([[u_x], [u_y]])

        # Initialize the state(x,y,z,x',y',z')
        self.x = np.array([[0], [0], [0], [0]])

        # Define the State Transition Matrix A
        self.A = np.array([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        # Define the Control Input Matrix B
        self.B = np.array([[(self.dt**2)/2, 0],
                            [0, (self.dt**2)/2],
                            [self.dt, 0],
                            [0, self.dt]])
        # Define the Measurement Mapping Matrix
        # Assume that we are only measuring the position but not the velocity
        self.H = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0]])
        # Initialize the Process Noise Covariance
        self.Q = np.array([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
                            [0, (self.dt**4)/4, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, self.dt**2, 0],
                            [0, (self.dt**3)/2, 0, self.dt**2]]) * (std_acc**2)
        # Initialize the Measurement Noise Covariance
        self.R = np.array([[x_std_meas**2, 0],
                            [0, y_std_meas**2]])
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
        rospy.loginfo(f"{self.x}")
        return self.x[0:2] # return x,y
    
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
        return self.x[0:2]




class ImageToDistance:

    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.bridge = CvBridge()
        self.imu_data = Imu()
        self.measured_xy = Point()
        self.predicted_xy = Point()
        self.filtered_xy = Point()
        # KalmanFilter(dt, u_x, u_y, std_acc, x_std_meas, y_std_meas)
        self.KF = KalmanFilter(0.1, 1, 1, 1, 0.1, 0.1)
        self.dis = Float32MultiArray()
        self.mission = 0

        #Publisher
        self.image_pub = rospy.Publisher("/cv_image", Image, queue_size= 1)
        self.distance_pub = rospy.Publisher("/relative_distance", Float32MultiArray, queue_size=1)
        self.measured_xy_pub = rospy.Publisher("/measured_xy", Point, queue_size=1)
        self.predicted_xy_pub = rospy.Publisher("/filtered_xy", Point, queue_size=1)
        self.filtered_xy_pub = rospy.Publisher("/filtered_xy", Point, queue_size=1)

        # Subscriber
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)
        self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)

        # cameraMatrix and distortion coefficents
        # Camera intrinsic matrix [[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]]
        # self.cameraMatrix = np.array([[1059.899814, 0. , 640.0], [ 0.0 , 1059.899814, 360.0], [0.0 , 0.0 , 1.0]])
        self.cameraMatrix = np.array([[1075.150341, 0. , 640.0], [ 0.0 , 1075.150341, 360.0], [0.0 , 0.0 , 1.0]])
        self.distortion = np.array([[0.176173, -0.394370, -0.003991, 0.005109]])
        
        # aruco basic settings
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # aruco casade structure
        self.inner_marker_size = 0.325/6
        self.outer_marker_size = 0.325
        self.inner_objp = np.array([[0, 0, 0], [0, self.inner_marker_size, 0], [self.inner_marker_size, self.inner_marker_size, 0], [self.inner_marker_size, 0, 0]], dtype=np.float32)
        self.outer_objp = np.array([[0, 0, 0], [0, self.outer_marker_size, 0], [self.outer_marker_size, self.outer_marker_size, 0], [self.outer_marker_size, 0, 0]], dtype=np.float32)
        #========================================================================================================
        device_index = get_usb_device()
        if device_index is None:
            rospy.logwarn("Get device failed!")
        self.cap = cv2.VideoCapture(device_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -10)

    def mission_cb(self, msg):
        self.mission = msg.data

    def state_cb(self, msg):
        self.current_state = msg
    
    def pose_cb(self, msg):
        self.current_pose = msg

    def imu_cb(self, msg):
        self.imu_data = msg 

    def image_to_distance_cb(self, data):
       
        ret, cv_image = self.cap.read()
        height, width, _ = cv_image.shape

        if self.mission == 6:
            # convert the image
            cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # detect marker configuration
            corners, ids, rejectedImgPoints = self.detector.detectMarkers(cv_image_gray)
            
            inner_id = None
            outer_id = None
            inner_corners = None
            outer_corners = None

            if np.all(ids != None):
                rospy.loginfo(f"AruCo Marker # {ids.size}")
                # We have cascade AruCo outer_id: 19 innter_id: 0
                for i in range(ids.size):
                    if outer_id is None:
                        outer_id = ids[i][0]
                        outer_corners = corners[i]

                    if ids.size > 1 and i > outer_id:
                        inner_id = outer_id
                        inner_corners = outer_corners
                        outer_id = ids[i][0]
                        outer_corners = corners[i]
                rospy.loginfo(f"AruCo Marker ID inner_id: {inner_id}, outer_id: {outer_id}")
                
            if inner_id is not None:
                tmp_corner = inner_corners[0]
                ret, rvec, tvec = cv2.solvePnP(self.inner_objp, tmp_corner, self.cameraMatrix, self.distortion)
            
            elif outer_id is not None:
                tmp_corner = outer_corners[0]
                ret, rvec, tvec = cv2.solvePnP(self.outer_objp, tmp_corner, self.cameraMatrix, self.distortion)

            else:
                self.dis = Float32MultiArray()
                self.dis.data = (float('nan'), float('nan'), float('nan'))
                self.distance_pub.publish(self.dis)

            if(corners):
                # detect marker
                aruco.drawDetectedMarkers(cv_image, corners)

                x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
                y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
                    
                x_center_px = x_sum*.25
                y_center_px = y_sum*.25
                self.measured_xy.x = x_center_px
                self.measured_xy.y = y_center_px
                self.measured_xy_pub.publish(self.measured_xy)
                # rospy.loginfo(f"pixel x: {x_center_px}, y: {y_center_px}")

                center = [[x_center_px], [y_center_px]]
                # rospy.loginfo(f"pixel x: {x_center_px}, y: {y_center_px}")
                
                # Predict
                (x_predict, y_predict) = self.KF.predict()
                self.predicted_xy.x = x_predict[0]
                self.predicted_xy.y = y_predict[0]
                self.predicted_xy_pub.publish(self.predicted_xy)
                # rospy.loginfo(f"predict x: {x_predict}, y: {y_predict}")
                cv2.rectangle(cv_image, (int(x_predict-30), int(y_predict-30)), (int(x_predict+30), int(y_predict+30)), (255,0,0), 2)

                # Update
                (x_update, y_update) = self.KF.update(center)
                self.filtered_xy.x = x_update[0]
                self.filtered_xy.y = y_update[0]
                self.filtered_xy_pub.publish(self.filtered_xy)
                cv2.rectangle(cv_image, (int(x_update-30), int(y_update-30)), (int(x_update+30), int(y_update+30)), (0,0,255), 2)

                cv2.putText(cv_image, "Estimated Position", (int(x_update + 30), int(y_update + 10)), 0, 0.5, (0, 0, 255), 2)
                cv2.putText(cv_image, "Predicted Position", (int(x_predict + 30), int(y_predict)), 0, 0.5, (255, 0, 0), 2)
                cv2.putText(cv_image, "Measured Position", (int(x_center_px + 30), int(y_center_px - 30)), 0, 0.5, (0,191,255), 2)


                #=========================Camera coordinate============================
                # Initialize camera_coord in pixels
                camera_coord = np.array([[x_update[0]], [y_update[0]], [1]])
                # Perform matrix inversion and multiplication
                camera_coord = np.linalg.inv(self.cameraMatrix).dot(camera_coord)
                # Multiply with z_world to get the coordinates in units of z_world
                camera_coord *= tvec[2][0]

                camera_coord = np.array([camera_coord[0][0], camera_coord[1][0], camera_coord[2][0], 1])
                rospy.loginfo(f"camera_coord: {camera_coord}")

                x = -camera_coord[0]
                y = camera_coord[1]
                z = camera_coord[2]
            

                self.dis.data = (x, y, z)
                
                # Node publish - pose information
                self.distance_pub.publish(self.dis)

        # Node publish - cv_image
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
        # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # cv2.imshow("cv_image", cv_image)
        # cv2.waitKey(5)

if __name__ == "__main__":
    rospy.init_node('aruco_VIO', anonymous=True)

    try:
        vision_kalman_filter_node_handler = ImageToDistance()

        rate = rospy.Rate(100)
        #wait for FCU connection
        while not rospy.is_shutdown() and not vision_kalman_filter_node_handler.current_state.connected:
            rate.sleep()
        rospy.loginfo("aruco VIO node : FCU connected")
        rospy.Timer(rospy.Duration(0.1), vision_kalman_filter_node_handler.image_to_distance_cb)

        rospy.spin()
    except KeyboardInterrupt:
        print('shut down')
    cv2.destroyAllWindows()

