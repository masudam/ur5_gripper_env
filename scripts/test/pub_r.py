#!/usr/bin/env python
import rospy
import sys
import time
#import cv2
#from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import numpy as np

class watchAction():
    def __init__(self):
        self.node_name = "watch_action"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.ac_sub = rospy.Subscriber("/actions", String, self.callback, queue_size=1)
        self.now_time = time.time()
        self.isTrue = True

    def callback(self, data):
        called_time = time.time()

        #print("time for 1 action : {}".format(called_time-self.now_time))
        self.now_time = called_time

    def cleanup(self):
        print("\nExiting...")
        rospy.signal_shutdown("Quit")
        self.isTrue = False
        #sys.exit()


    def watch(self):
        pub = rospy.Publisher('/actions', String, queue_size=10)
        
        while self.isTrue:
            print("check...")
            dif = time.time()-self.now_time
            if dif > 600.0:
                print("maybe wait fot action")
                pub.publish("recover")
                self.now_time = time.time()
            else:
                print("time is ok!")
            time.sleep(60)


if __name__ == '__main__':
    try:
      wa = watchAction()
      wa.watch()
      #rospy.spin()
    except:
        rospy.signal_shutdown("Quit")
        rospy.spin()
        #sys.exit()








