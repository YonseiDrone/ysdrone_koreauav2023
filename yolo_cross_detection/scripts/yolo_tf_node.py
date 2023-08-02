#!/usr/bin/env python3

import rospy, rospkg
import cv2, torch
import numpy as np
from sensor_msgs.msg import Image
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


camera_center = [0, 0, 0]

class MarkerDetection(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.yoloPath = self.rospack.get_path('yolo_cross_detection') + '/yolov5'
        self.weightPath = self.rospack.get_path('yolo_cross_detection') + '/weight/yolov5nV4.onnx'
        self.model = torch.hub.load(self.yoloPath, 'custom', self.weightPath, source='local', force_reload=True)
        self.model.iou = 0.5
        self.mission = 0

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.final_coord = PoseStamped()

        # Subscriber
        self.rgb_image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_image_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)



        # Synchronize the topics
        ts = message_filters.ApproximateTimeSynchronizer([self.rgb_image_sub, self.depth_image_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.image_cb)

        # Publisher
        self.image_pub = rospy.Publisher('/cross_image', Image, queue_size=1)
        self.final_coord_pub = rospy.Publisher('/marker_position/home', PoseStamped, queue_size=1)
    
    def mission_cb(self, msg):
        self.mission = msg.data

    def state_cb(self, msg):
        prev_state = self.current_state
        self.current_state = msg

        if self.current_state.mode != prev_state.mode:
            rospy.loginfo(f"Current Mode : {self.current_state.mode}")
        if self.current_state.armed != prev_state.armed:
            rospy.loginfo(f"Vehicle armed : {self.current_state.armed}")
    
    def pose_cb(self, msg):
        self.current_pose = msg

    def get_3d_coord(self, pixel_x, pixel_y, depth_frame):
        depth_frame = depth_frame

        distance = depth_frame[int(pixel_y), int(pixel_x)] # (y, x)
        pixel_center = np.array([int(pixel_x), int(pixel_y)]) # (x, y)

        #===============================Camera coordinate==========================================
        intrinsic_matrix = np.array([[385.7627868652344, 0.0, 331.9479064941406],
                    [0.0, 385.4613342285156, 237.6436767578125],
                    [0.0, 0.0, 1.0]])
        camera_center[2] = distance * 0.001 #z
        camera_center[0] = (pixel_center[0] - intrinsic_matrix[0,2]) * camera_center[2] / intrinsic_matrix[0, 0] #x
        camera_center[1] = (pixel_center[1] - intrinsic_matrix[1,2]) * camera_center[2] / intrinsic_matrix[1, 1] #y
        camera_coord = np.array([camera_center[0], camera_center[1], camera_center[2], 1])
        rospy.loginfo(f"camera_coord: {camera_coord}")
        #===============================FLU coordinate(front-left-up)==========================================
        x_rotation = 90 * math.pi /180
        y_rotation = -90 * math.pi /180
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
        rospy.loginfo(f"flu_coord: {flu_coord}")
        #==================================ENU coordinate(east-north-up)=======================================
        _, _, yaw = to_euler_angles(self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w)

        enu_rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        enu_translation = np.array([0, 0, 0])

        flu_to_enu = np.eye(4)
        flu_to_enu[:3, :3] = enu_rotation
        flu_to_enu[:3, 3] = enu_translation

        enu_coord = np.dot(flu_to_enu, flu_coord)
        rospy.loginfo(f"enu_coord: {enu_coord}")
        #=============================Local coordinate(home position)===============
        final_coord_x = self.current_pose.pose.position.x + enu_coord[0]
        final_coord_y = self.current_pose.pose.position.y + enu_coord[1]
        final_coord_z = self.current_pose.pose.position.z + enu_coord[2]

        rospy.loginfo(f"home_coord: {final_coord_x}, {final_coord_y}, {final_coord_z}")
        self.final_coord.pose.position.x = final_coord_x
        self.final_coord.pose.position.y = final_coord_y
        self.final_coord.pose.position.z = final_coord_z
        self.final_coord_pub.publish(self.final_coord)
    

    def image_cb(self, rgb_image, depth_image):
        bridge = CvBridge()

        try:
            if self.mission == 3:

                rgb_frame = bridge.imgmsg_to_cv2(rgb_image, desired_encoding="rgb8")
                depth_frame = bridge.imgmsg_to_cv2(depth_image, desired_encoding="16UC1")
                resolution = (rgb_frame.shape[0], rgb_frame.shape[1])
                results = self.model(cv2.resize(rgb_frame, (640, 640)))
                xyxy = None # Initialize xyx with None

                for volume in results.xyxy[0]:
                    xyxy = volume.numpy()
                    #resize
                    xyxy[0] = xyxy[0] / 640 * resolution[0]
                    xyxy[2] = xyxy[2] / 640 * resolution[0]
                    xyxy[1] = xyxy[1] / 640 * resolution[1]
                    xyxy[3] = xyxy[3] / 640 * resolution[1]

                    rospy.loginfo(f"Pixel Coordinate | x: {(xyxy[0] + xyxy[2])/2}, y: {(xyxy[1] + xyxy[3])/2}")
                    cv2.rectangle(rgb_frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color=(0, 255, 0), thickness=2)
                    cv2.putText(rgb_frame, f'{xyxy[4]:.3f}', (int(xyxy[0]), int(xyxy[1])), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 255, 0), thickness=2)
                    pixel_x = (xyxy[0] + xyxy[2])/2
                    pixel_y = (xyxy[1] + xyxy[3])/2

                if xyxy is not None:
                    self.get_3d_coord(pixel_x, pixel_y, depth_frame)

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