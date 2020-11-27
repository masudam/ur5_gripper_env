#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ContactsState

class tac_watcher():
    def __init__(self,tac_name):
        self.touch_inside = 0
        self.touch_outside = 0
        self.tac_name = tac_name
        self.inside_touch_pub = rospy.Publisher("contact_inside", String, queue_size=10)
        self.outside_touch_pub = rospy.Publisher("contact_outside", String, queue_size=10)

    def ac_callback(self,data):
        print("new action is {}".format(data.data))
        print("flag reset")
        self.touch_inside = 0
        self.touch_outside = 0

    def inside_tac_callback(self,data):
        if not(data.states == []):
            self.touch_inside = 1
            #print(data.states)

    def outside_tac_callback(self,data):
        if not(data.states == []):
            self.touch_outside = 1
            #print(data.states)

    def reset_flag(self,data):
        print("flag reset!!")
        self.touch_inside=0
        self.touch_outside=0

    def listener(self):
        print("!!start spin!!")
        rate = rospy.Rate(50)

        rospy.Subscriber('actions', String, self.ac_callback)
        rospy.Subscriber('reset_touch_flag', String, self.reset_flag)
        rospy.Subscriber(self.tac_name[0], ContactsState, self.inside_tac_callback)
        rospy.Subscriber(self.tac_name[1], ContactsState, self.inside_tac_callback)
        #rospy.Subscriber(self.tac_name[2], ContactsState, self.outside_tac_callback)
        #rospy.Subscriber(self.tac_name[3], ContactsState, self.outside_tac_callback)

        cc=0
        while not rospy.is_shutdown():
            cc+=1
            self.inside_touch_pub.publish(str(self.touch_inside))
            self.outside_touch_pub.publish(str(self.touch_outside))
            rate.sleep()
            if cc % 250 == 0:
                cc =0
                print("contact_inside : {}, contact_outside : {}".format(self.touch_inside,self.touch_outside))



if __name__ == '__main__':
    rospy.init_node("tac_test")
    tac_name_list = ["right_tip_inside_contact_state", "left_tip_inside_contact_state"]
    try:
        tc = tac_watcher(tac_name_list)
        tc.listener()
    except rospy.ROSInterruptException:
        pass


