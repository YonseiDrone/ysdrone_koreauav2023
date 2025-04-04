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
    '''
    Function to fix the device number on the USB camera
    '''
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



class KalmanFilter(object):
    def __init__(self, dt, u_x, u_y, std_acc, x_std_meas, y_std_meas):
        """
        Linear kalman filter to reduce camera sensor noise.
        Inputs:
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
    '''
    A class to determine the current position of the UAV based on the ENU coordinated system. 
    The output is the position of UAV based on the aruco marker with a casade structure. In this case, three aruco markers of different sizes were used.
    '''

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
        self.cameraMatrix = np.array([[1075.150341, 0. , 640.0], [ 0.0 , 1075.150341, 360.0], [0.0 , 0.0 , 1.0]])
        self.distortion = np.array([[0.176173, -0.394370, -0.003991, 0.005109]])
        
        # aruco basic settings
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # aruco casade structure
        self.inner_marker_size = 0.08
        self.mid_marker_size = 0.45
        self.outer_marker_size = 1.92
        self.inner_objp = np.array([[0, 0, 0], [0, self.inner_marker_size, 0], [self.inner_marker_size, self.inner_marker_size, 0], [self.inner_marker_size, 0, 0]], dtype=np.float32)
        self.mid_objp = np.array([[0, 0, 0], [0, self.mid_marker_size, 0], [self.mid_marker_size, self.mid_marker_size, 0], [self.mid_marker_size, 0, 0]], dtype=np.float32)
        self.outer_objp = np.array([[0, 0, 0], [0, self.outer_marker_size, 0], [self.outer_marker_size, self.outer_marker_size, 0], [self.outer_marker_size, 0, 0]], dtype=np.float32)
        
        # Fix USB camera number
        device_index = get_usb_device()
        if device_index is None:
            rospy.logwarn("Get device failed!")
        self.cap = cv2.VideoCapture(device_index)

        # Resolution(1280x720) fixed because camera calibration was performed at 1280x720 resolution.
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

        '''
        This class is used to determine the distance from the aruco marekr to the UAV,
        but there are two ways(PID or RL) to control the UAV from the obtained distance, so it works with two mission(6, 10) numbers.
        '''
        if self.mission in [6, 10]:
            # convert the image
            cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # detect marker configuration
            corners, ids, rejectedImgPoints = self.detector.detectMarkers(cv_image_gray)
            
            inner_id = None
            outer_id = None
            mid_id = None
            inner_corners = None
            mid_corners = None
            outer_corners = None

            '''
            Detect all recognized aruco marker IDs.
            '''
            if np.all(ids != None):
                rospy.loginfo(f"AruCo Marker # {ids.size}")
                # We have cascade AruCo outer_id: 19 mid_id: 1 inner_id: 0
                for i in range(ids.size):
                    if ids[i][0] == 19:
                        outer_id = ids[i][0]
                        outer_corners = corners[i]
                    # The inner marker has an ID of 1 now
                    if ids[i][0] == 1:
                        mid_id = ids[i][0]
                        mid_corners = corners[i]
                    if ids[i][0] == 0:
                        inner_id = ids[i][0]
                        inner_corners = corners[i]
                        
                rospy.loginfo(f"AruCo Marker ID inner_id: {inner_id}, mid_id: {mid_id} outer_id: {outer_id}")

            '''
            The code structure below determines the distance based on the inner if it is recognized.
            However, if the distance is large and the outer, larger size is recognized instead of the inner, the distance is judged based on that.
            '''
                
            if inner_id is not None:
                tmp_corner = inner_corners[0]
                ret, rvec, tvec = cv2.solvePnP(self.inner_objp, tmp_corner, self.cameraMatrix, self.distortion)
                rospy.loginfo("Inner")
            
            elif mid_id is not None:
                tmp_corner = mid_corners[0]
                ret, rvec, tvec = cv2.solvePnP(self.mid_objp, tmp_corner, self.cameraMatrix, self.distortion)
                rospy.loginfo("Mid")
            
            elif outer_id is not None:
                tmp_corner = outer_corners[0]
                ret, rvec, tvec = cv2.solvePnP(self.outer_objp, tmp_corner, self.cameraMatrix, self.distortion)
                rospy.loginfo("Outer")

            else:
                self.dis = Float32MultiArray()
                self.dis.data = (float('nan'), float('nan'), float('nan'))
                self.distance_pub.publish(self.dis)

            '''
            Calculate the distance to the UAV using the recognized aruco marker.
            In this process, the kalman filter is used to reduce sensor noise.
            '''
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

                # Initialize camera_coord in pixels
                camera_coord = np.array([[x_update[0]], [y_update[0]], [1]])

                # Perform matrix inversion and multiplication
                camera_coord = np.linalg.inv(self.cameraMatrix).dot(camera_coord)

                # Multiply with z_world to get the coordinates in units of z_world
                camera_coord *= tvec[2][0]

                camera_coord = np.array([camera_coord[0][0], camera_coord[1][0], camera_coord[2][0], 1])
                #rospy.loginfo(f"camera_coord: {camera_coord}")

                '''
                In our case, to simplify the coordinate transformation process, UAV will land with the yaw direction fixed at 90 degrees during whole landing process.
                Therefore, in this process, set x,y and z as below to match the camera coordinate systemd and the ENU coordinate system.
                '''
                x = -camera_coord[0]
                y = camera_coord[1]
                z = camera_coord[2]
            

                self.dis.data = (x, y, z)
                
                # Node publish - pose information
                self.distance_pub.publish(self.dis)

        # Node publish - cv_image
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))

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

