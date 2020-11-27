#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('actions', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(0.25)
    count = 0

    while not rospy.is_shutdown():
        str_co = str(count)
        asd = raw_input("input number :")
        if str(asd) == "R":
            asd = "recover"
        elif str(asd) == "r":
            asd = "reset"
        str_co = str(asd)
        rospy.loginfo(str_co)
        pub.publish(str_co)
        count = (count + 1) % 7

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

