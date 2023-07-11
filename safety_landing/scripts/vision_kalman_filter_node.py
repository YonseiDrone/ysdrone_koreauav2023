#!/usr/bin/env python3

from __future__ import print_function
import time
import roslib
import sys
import rospy
import cv2
import glob
import numpy as np
import cv2.aruco as aruco
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.msg import State

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
        self.u = np.matrix([[u_x], [u_y], [u_z]])

        # Initialize the state(x,y,z,x',y',z')
        self.x = np.matrix([[0], [0], [0], [0], [0], [0]])

        # Define the State Transition Matrix A
        self.A = np.matrix([[1, 0, 0, self.dt, 0 ,0],
                            [0, 1, 0, 0, self.dt, 0],
                            [0, 0, 1, 0, 0, self.dt],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])
        # Define the Control Input Matrix B
        self.B = np.matrix([[(self.dt**2)/2, 0, 0],
                            [0, (self.dt**2)/2, 0],
                            [0, 0, (self.dt**2)/2],
                            [self.dt, 0, 0],
                            [0, self.dt, 0],
                            [0, 0, self.dt]])
        # Define the Measurement Mapping Matrix
        # Assume that we are only measuring the position but not the velocity
        self.H = np.matrix([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])
        # Initialize the Process Noise Covariance
        self.Q = np.matrix([[(self.dt**4)/4, 0, 0, (self.dt**3)/2, 0, 0],
                            [0, (self.dt**4)/4, 0, 0, (self.dt**3)/2, 0],
                            [0, 0, (self.dt**4)/4, 0, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, 0, self.dt**2, 0, 0],
                            [0, (self.dt**3)/2, 0, 0, self.dt**2, 0],
                            [0, 0, (self.dt**3)/2, 0, 0, self.dt**2]]) * (std_acc**2)
        # Initialize the Measurement Noise Covariance
        self.R = np.matrix([[x_std_meas, 0, 0],
                            [0, y_std_meas, 0],
                            [0, 0, z_std_meas]])
        # Initialize the Covariance Matrix
        # identity matrix whose shape is the same as the shape of the matrix A
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
        self.P = (I - (K*self.H)) * self.P
        return self.x[0:3]




class ImageToDistance:

    def __init__(self):
        self.lostnumber = 0
        self.current_state = State()
        self.bridge = CvBridge()
        # Create KalmanFilter object as KF
        # KalmanFilter(dt, u_x, u_y, u_z, std_acc, x_std_meas, y_std_meas, z_std_meas)
        self.KF = KalmanFilter(0.1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001)

        #Publisher
        self.image_pub = rospy.Publisher("/cv_image", Image, queue_size= 5)
        self.distance_pub = rospy.Publisher("/relative_distance", Float32MultiArray, queue_size=1)

        #========================================================================================================
        #Subscriber
        # 나중에 하방 카메라 토픽에 맞춰서 변경하자!!!!!!!
        self.image_sub = rospy.Subscriber("/usb_camera/image_raw", Image, self.image_to_distance_cb)
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        # cameraMatrix and distortion coefficents
        # Camera intrinsic matrix [[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]]
        self.mtx = np.array([[238.3515418007097, 0. , 200.5], [ 0.0 , 238.3515418007097, 200.5], [0.0 , 0.0 , 1.0]])
        self.dist = np.array([[0, 0, 0, 0, 0]])
        #========================================================================================================

    def state_cb(self, msg):
        self.current_state = msg
    
    def image_to_distance_cb(self, data):
        try:
            # transform ROS image message into opencv image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        # aruco basic setting
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)

        # hese parameters include things like marker detection thresholds, corner refinement methods, and adaptive thresholding parameters
        # we should change these parameters so that can achieve the desired level of marker detection accuracy and robustness
        parameters = aruco.DetectorParameters_create()

        # convert the image
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # detect marker configuration
        corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image_gray, aruco_dict, parameters=parameters)

        if np.all(ids != None):
            self.lostnumber = 0
            id = ids[0][0]
            lock_number = 0
            for i in range(ids.size):
                if ids[i][0] > id:
                    id = ids[i][0]
                    lock_number = i
        #========================================================================================================
        # Marker_size에 맞게 나중에 조절!!
            marker_size = 0

            if id==1:
                marker_size = 0.139
            elif id==0:
                marker_size = 1
            elif id==2:
                marker_size = 0.071
            elif id==3:
                marker_size = 0.0325
            elif id==4:
                marker_size = 0.016
        #========================================================================================================

            # pose estimation
            # mtx: cameraMatrix, dist: distortion coefficients
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[lock_number], marker_size, self.mtx, self.dist)
            

            # read corners information
            top_left_X = corners[0][0][0][0]
            top_left_Y = corners[0][0][0][1]

            top_right_X = corners[0][0][1][0]
            top_right_Y = corners[0][0][1][1]

            bottom_left_X = corners[0][0][2][0]
            bottom_left_Y = corners[0][0][2][1]

            bottom_right_X = corners[0][0][3][0]
            bottom_right_Y = corners[0][0][3][1]

            # Predict
            (x, y, z) = self.KF.predict()
            # cv2.rectangle(cv_image, (int(x - 15), int(y - 15)), (int(x + 15), int(y + 15)), (255, 0, 0), 2)
            # cv2.putText(cv_image, "Predicted Position", (int(x+15), int(y)), 0, 0.5, (255,0,0),2)

            # get pose information
            cor_x = tvec[0][0][0]
            cor_y = tvec[0][0][1]
            cor_z = tvec[0][0][2]

            cor_xyz = np.array([[cor_x], [cor_y], [cor_z]])
            print(cor_xyz)
            

            print("Before x=", cor_x)
            print("Before y=", cor_y)
            print("Before z=", cor_z)

            # Update
            (x1, y1, z1) = self.KF.update(cor_xyz)
            # cv2.rectangle(cv_image, (int(x1-15), int(y1-15)), (int(x1+15), int(y1+15)), (0, 0, 255), 2)
            # cv2.putText(cv_image, "Estimated Position", (int(x1+15), int(y1)), 0, 0.5, (0,0,255),2)
            print("After x=", x1)
            print("After y=", y1)
            print("After z=", z1)

        

            # draw axis and detect marker
            #aruco.drawAxis(cv_image, mtx, dist, rvec[0], tvec[0], 0.01)
            aruco.drawDetectedMarkers(cv_image, corners)

            # draw ID text on top of image
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_image, "X: {}".format(cor_x), (0,364), font, 1, (0,255,0), 2, cv2.LINE_AA)
            cv2.putText(cv_image, "Y: {}".format(cor_y), (0, 400), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(cv_image, "Z: {}".format(cor_z), (0, 436), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # incorporate pose information together and print on image
            dis = Float32MultiArray()

            # 기존의 cor_x, cor_y, cor_z에서 estimated value로 바꿈!
            dis.data = (x1, y1, z1)
            
            # # Debugging
            # rospy.loginfo(f"x : {dis.data[0][0]}, y: {dis.data[1][0]}, z: {dis.data[2][0]}")
            rospy.loginfo(f"dis.data : {dis.data}")

            cv2.imshow("Image", cv_image)
            cv2.waitKey(3)

            # Node publish - pose information
            self.distance_pub.publish(dis)

            # Node publish - cv_image
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)
        else:
            self.lostnumber += 1
            if self.lostnumber > 100:
                dis = Float32MultiArray()
                dis.data = (float('nan'), float('nan'), float('nan'))
                self.distance_pub.publish(dis)

if __name__ == "__main__":
    rospy.init_node('vision_kalman_filter_node', anonymous=True)

    try:
        vision_kalman_filter_node_handler = ImageToDistance()

        rate = rospy.Rate(100)
        # wait for FCU connection
        while not rospy.is_shutdown() and not vision_kalman_filter_node_handler.current_state.connected:
            rate.sleep()
        rospy.loginfo("vision kalman filter node : FCU connected")

        rospy.spin()
    except KeyboardInterrupt:
        print('shut down')
    cv2.destroyAllWindows()

