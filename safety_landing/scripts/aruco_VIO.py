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
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
import math


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
    def __init__(self, dt, u_x, u_y, u_z, std_acc, x_std_meas, y_std_meas, z_std_meas):
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
        self.u = np.matrix([[u_x], [u_y]])

        # Initialize the state(x,y,z,x',y',z')
        self.x = np.matrix([[0], [0], [0], [0]])

        # Define the State Transition Matrix A
        self.A = np.matrix([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        # Define the Control Input Matrix B
        self.B = np.matrix([[(self.dt**2)/2, 0],
                            [0, (self.dt**2)/2],
                            [self.dt, 0],
                            [0, self.dt]])
        # Define the Measurement Mapping Matrix
        # Assume that we are only measuring the position but not the velocity
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])
        # Initialize the Process Noise Covariance
        self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
                            [0, (self.dt**4)/4, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, self.dt**2, 0],
                            [0, (self.dt**3)/2, 0, self.dt**2]]) * (std_acc**2)
        # Initialize the Measurement Noise Covariance
        self.R = np.matrix([[x_std_meas, 0],
                            [0, y_std_meas]])
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
        return self.x[0:2] # return x,y,z
    
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
        return self.x[0:2]




class ImageToDistance:

    def __init__(self):
        self.lostnumber = 0
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.bridge = CvBridge()
        self.imu_data = Imu()
        # Create KalmanFilter object as KF
        self.KF = KalmanFilter(0.1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001)

        #Publisher
        self.image_pub = rospy.Publisher("/cv_image", Image, queue_size= 1)
        self.distance_pub = rospy.Publisher("/relative_distance", Float32MultiArray, queue_size=1)

        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)
        # cameraMatrix and distortion coefficents
        # Camera intrinsic matrix [[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]]
        self.cameraMatrix = np.array([[1347.578105, 0. , 640.0], [ 0.0 , 1347.578105, 360.0], [0.0 , 0.0 , 1.0]])
        self.distortion = np.array([[-0.042381, 0.122926, -0.003833, -0.017073]])
        
        # aruco basic setting
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        #========================================================================================================
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def state_cb(self, msg):
        self.current_state = msg
    
    def pose_cb(self, msg):
        self.current_pose = msg

    def imu_cb(self, msg):
        self.imu_data = msg 

    def image_to_distance_cb(self, data):
       
        ret, cv_image = self.cap.read()
        height, width, _ = cv_image.shape

        marker_size = 0.11
        objp = np.array([[0, 0, 0], [0, marker_size, 0], [marker_size, marker_size, 0], [marker_size, 0, 0]], dtype=np.float32)

        # hese parameters include things like marker detection thresholds, corner refinement methods, and adaptive thresholding parameters
        # we should change these parameters so that can achieve the desired level of marker detection accuracy and robustness
        #parameters = aruco.DetectorParameters_create()

        # convert the image
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # detect marker configuration
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(cv_image_gray)

        if np.all(ids != None):
            self.lostnumber = 0
            id = ids[0][0]
            lock_number = 0
            for i in range(ids.size):
                if ids[i][0] > id:
                    id = ids[i][0]
                    lock_number = i

            corners2 = corners[0][0]

            ret, rvec, tvec = cv2.solvePnP(objp, corners2, self.cameraMatrix, self.distortion)
            # draw axis and detect marker
            #aruco.drawAxis(cv_image, mtx, dist, rvec[0], tvec[0], 0.01)
            aruco.drawDetectedMarkers(cv_image, corners)

            # incorporate pose information together and print on image
            dis = Float32MultiArray()

            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
                
            x_center_px = x_sum*.25
            y_center_px = y_sum*.25
            #rospy.loginfo(f"pixel x: {x_center_px}, y: {y_center_px}")

            #=========================Camera coordinate============================
            # Initialize camera_coord in pixels
            camera_coord = np.array([[x_center_px], [y_center_px], [1]])
            # Perform matrix inversion and multiplication
            camera_coord = np.linalg.inv(self.cameraMatrix).dot(camera_coord)
            # Multiply with z_world to get the coordinates in units of z_world
            camera_coord *= tvec[2][0]

            camera_coord = np.array([camera_coord[0][0], camera_coord[1][0], camera_coord[2][0], 1])
            rospy.loginfo(f"camera_coord: {camera_coord}")

            x = -camera_coord[0][0]
            y = camera_coord[1][0]
            z = camera_coord[2][0]
        

            dis.data = (x, y, z)
            
            # Node publish - pose information
            self.distance_pub.publish(dis)

        else:
            self.lostnumber += 1
            if self.lostnumber > 100:
                dis = Float32MultiArray()
                dis.data = (float('nan'), float('nan'))
                self.distance_pub.publish(dis)
        	# Node publish - cv_image
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(5)

if __name__ == "__main__":
    rospy.init_node('aruco_vision_node', anonymous=True)

    try:
        vision_kalman_filter_node_handler = ImageToDistance()

        rate = rospy.Rate(100)
        # wait for FCU connection
        #while not rospy.is_shutdown() and not vision_kalman_filter_node_handler.current_state.connected:
        #rate.sleep()
        rospy.loginfo("aruco vision node : FCU connected")
        rospy.Timer(rospy.Duration(0.1), vision_kalman_filter_node_handler.image_to_distance_cb)

        rospy.spin()
    except KeyboardInterrupt:
        print('shut down')
    cv2.destroyAllWindows()

