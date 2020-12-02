#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

class cvBridgeDemo:
    def __init__(self):
        self.node_name = "cv_bridge_demo_compressed"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.image_sub = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1)

    def image_callback(self, ros_image_compressed):
        try:
            np_arr = np.fromstring(ros_image_compressed.data, np.uint8)
            input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #print(input_image.shape)
        except CvBridgeError:
            print("Error")

        cv2.imshow(self.node_name, input_image[90:310,25:375])
        cv2.waitKey(1)

    def cleanup(self):
        cv2.destroyAllWindows() 


if __name__ == '__main__':
    cvBridgeDemo()
    rospy.spin()


