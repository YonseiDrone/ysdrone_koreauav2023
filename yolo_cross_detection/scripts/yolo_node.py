#!/usr/bin/env python3

import rospy, rospkg
import cv2, torch
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from ysdrone_msgs.srv import *
import time


class MarkerDetection(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.yoloPath = self.rospack.get_path('yolo_cross_detection') + '/yolov5'
        self.weightPath = self.rospack.get_path('yolo_cross_detection') + '/weight/yolov5nV4.pt'
        self.model = torch.hub.load(self.yoloPath, 'custom', self.weightPath, source='local')
        self.model.iou = 0.5
        self.mission = 0

        # Subscriber
        self.mission_sub = rospy.Subscriber('/mission', Float32, self.mission_cb)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_subscriber)

        # Publisher
        self.image_pub = rospy.Publisher('/cross_image', Image, queue_size=10)


    def mission_cb(self, msg):
        self.mission = msg.data

    def image_subscriber(self, image_msg):
        bridge = CvBridge()

        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환합니다.
            if self.mission == 3:
                #fps
                timer = time.time()

                frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
                xyxy = None # Initialize xyz with None
                results = self.model(frame)

                for volume in results.xyxy[0]:
                    xyxy = volume.numpy()
                    cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color=(0, 255, 0), thickness=2)
                    cv2.putText(frame, f'{xyxy[4]:.3f}', (int(xyxy[0]), int(xyxy[1])), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 255, 0), thickness=2)
                    
                    #fps
                    dt = time.time() - timer
                    fps = 1 / dt
                    cv2.putText(frame, f'FPS:{fps:.2f}-{dt*1000:.0f}ms', (0, 100), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 0, 0), thickness=3)
                #rospy.loginfo(f"image coordinate: {(int(xyxy[0]) + int(xyxy[2]))/2, (int(xyxy[1]) + int(xyxy[3]))/2}")
            try:
                self.image_pub.publish(bridge.cv2_to_imgmsg(frame, "rgb8"))
            except CvBridgeError as e:
                print(e)
            
            cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cv2.imshow("Received Image", frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == '__main__':
    try:
        rospy.init_node('yolo_node', anonymous=True)
        rospy.loginfo("yolo_node : FCU connected")
        marker_detection_node = MarkerDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
