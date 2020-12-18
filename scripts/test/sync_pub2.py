#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('test2', String, queue_size=10)
    rospy.init_node('sync_pub2', anonymous=True)
    r = rospy.Rate(10)
    count = 0

    while not rospy.is_shutdown():
        str_co = str(count)
        rospy.loginfo(str_co)
        pub.publish(str_co)
        count = (count + 1) % 7
        r.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass






