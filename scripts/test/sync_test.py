#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import message_filters

class sync_test():
    def __init__(self):
        rospy.init_node('sync_test', anonymous=True)
        sub1 = message_filters.Subscriber('test1', String)
        sub2 = message_filters.Subscriber('test2', String)

        fps = 10.
        delay = 1/fps
        ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 10, delay, allow_headerless=True)
        ts.registerCallback(self.callback)


    def callback(self, msg1, msg2):
        print("msg1 is {},  msg2 is {}".format(msg1.data, msg2.data))


    def talker():
        pub = rospy.Publisher('test', String, queue_size=10)
        r = rospy.Rate(0.25)
        count = 0

        while not rospy.is_shutdown():
            str_co = str(count)
            asd = raw_input("input number :")
            str_co = str(asd)
            rospy.loginfo(str_co)
            pub.publish(str_co)
            count = (count + 1) % 7
            #break



if __name__ == '__main__':
    try:
        syn = sync_test()
        rospy.spin()
    except rospy.ROSInterruptException: pass










