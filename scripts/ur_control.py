#coding:UTF-8
import argparse
import math
import random
import numpy
import sys
from time import sleep
import cv2
import numpy as np
import pickle

import rospy
import roslib
import rospkg
from cv_bridge import CvBridge

from ur5_robot_class import UrManipulator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--recover", help="optional", action="store_true")
    args = parser.parse_args()

    print("Initializing node... ")
    rospy.init_node("ur_control")

    ur_manipulator = UrManipulator()
    print("mainpulator ready")

    ur_manipulator.reset_env()

    if args.recover:
        cmd = "recover"
        ur_manipulator.recover_env()

    try:
        ur_manipulator.listener()
    except SystemExit as ex:
        sys.exit()
    except:
        #baxter_manipulator.recover_func()
        rospy.signal_shutdown("Quit")
        sys.exit()




if __name__ == '__main__':
    main()


