#!/usr/bin/env python3

import rospy, rospkg
import cv2, torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


rospack = rospkg.RosPack()
yoloPath = rospack.get_path('yolo_cross_detection') + '/yolov5'
weightPath = rospack.get_path('yolo_cross_detection') + '/weight/yolov5nV4.pt'

model = torch.hub.load(yoloPath, 'custom', weightPath, source='local')
model.iou = 0.5

def image_subscriber(image_msg):
    bridge = CvBridge()

    try:
        # ROS 이미지 메시지를 OpenCV 이미지로 변환합니다.
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
        results = model(frame)

        for volume in results.xyxy[0]:
            xyxy = volume.numpy()
            cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color=(0, 255, 0), thickness=2)
            cv2.putText(frame, f'{xyxy[4]:.3f}', (int(xyxy[0]), int(xyxy[1])), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 255, 0), thickness=2)

        # cv2.imshow("Received Image", frame)
        # cv2.waitKey(1)
        try:
            image_pub.publish(bridge.cv2_to_imgmsg(frame, "rgb8"))
        except CvBridgeError as e:
            print(e)

    except CvBridgeError as e:
        rospy.logerr(e)


if __name__ == '__main__':
    try:
        rospy.init_node('yolo_node', anonymous=True)
        image_pub = rospy.Publisher('/cross_image', Image, queue_size=10)
        # rospy.Subscriber('/usb_cam/image_raw', Image, image_subscriber)
        rospy.Subscriber('/camera/rgb/image_raw', Image, image_subscriber)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
