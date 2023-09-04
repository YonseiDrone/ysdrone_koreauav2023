import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import datetime

class ImageToVideoConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/cross_image', Image, self.image_callback)
        self.fps = 24
        self.video_writer = None
        self.last_frame_time = None
        self.datetime = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            print(e)
            return

        current_frame_time = rospy.Time.now()
        if self.last_frame_time is not None:
            time_diff = (current_frame_time - self.last_frame_time).to_sec()
            if self.video_writer is None:
                height, width, _ = cv_image.shape
                fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Video codec
                self.video_writer = cv2.VideoWriter(f'/home/khadas/output_video_{self.datetime}.avi', fourcc, self.fps, (width, height))
            
            num_frame = int(time_diff * self.fps)
            for _ in range(num_frame):
                self.video_writer.write(cv_image)
            self.last_frame_time = current_frame_time - rospy.Duration.from_sec(time_diff - (float(num_frame) / self.fps))
        else:
            self.last_frame_time = current_frame_time
        # print(f'Frame written to video.')

    def run(self):
        rospy.spin()
        if self.video_writer is not None:
            self.video_writer.release()
            rospy.loginfo('[Image Recorder] Video saved.')

if __name__ == '__main__':
    rospy.init_node('image_recorder', anonymous=True)
    converter = ImageToVideoConverter()
    converter.run()
