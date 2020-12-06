#coding:UTF-8
import argparse
import sys

import rospy
import roslib
import rospkg

from ur5_robot_class import UrManipulator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--recover", help="optional", action="store_true")
    args = parser.parse_args()

    print("Initializing node... ")
    rospy.init_node("ur_control")
    ur_manipulator = UrManipulator()
    ur_manipulator.reset_env()

    if args.recover:
        ur_manipulator.recover_env()

    try:
        ur_manipulator.listener()
    except SystemExit as ex:
        sys.exit()
    except:
        rospy.signal_shutdown("Quit")
        sys.exit()


if __name__ == '__main__':
    main()

