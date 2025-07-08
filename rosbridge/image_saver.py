#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self.saved = False

    def callback(self, msg):
        if not self.saved:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                img = cv_image.copy()
                cv2.imwrite('/home/er/Documents/reasoning320/googleResoning/current_frame.png', img)
                rospy.loginfo("Image saved to /home/er/Documents/reasoning320/googleResoning/current_frame.png")
                self.saved = True
                rospy.signal_shutdown("Image saved")
            except Exception as e:
                rospy.logerr("Failed to save image: %s", e)

if __name__ == '__main__':
    ImageSaver()
    rospy.spin()