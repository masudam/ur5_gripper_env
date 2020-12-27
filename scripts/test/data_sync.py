#!/usr/bin/env python
import sys
import rospy
import numpy as np
import time
from std_msgs.msg import String
import message_filters
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.msg import ModelStates

FPS=30.

class dataSync():
    def __init__(self):
        rospy.init_node('data_sync', anonymous=True, disable_signals=True)
        #rospy.init_node('data_sync', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        hand_sub = message_filters.Subscriber('/hand_controller/state', JointTrajectoryControllerState)
        joint_sub = message_filters.Subscriber('/joint_states', JointState)
        obj_sub = message_filters.Subscriber('/gazebo/model_states', ModelStates)
        time_sub = message_filters.Subscriber('/my_timer', String)

        self.data=[]

        fps = FPS
        delay = 2/fps
        ts = message_filters.ApproximateTimeSynchronizer([hand_sub,joint_sub,obj_sub,time_sub], 10, delay, allow_headerless=True)
        ts.registerCallback(self.callback)

    def clean_shutdown(self):
        print("\nExiting...")
        rospy.signal_shutdown("Quit")
        #rospy.spin()

    def callback(self, hand_msg, joint_msg, obj_msg, time_msg):
        #print("now count is {}".format(time_msg.data))
        #print(obj_msg.name[2])
        obj_pos = obj_msg.pose[2].position
        obj_ori = obj_msg.pose[2].orientation
        #print(obj_pos)
        #print(obj_ori)

        arm_data = joint_msg.position + hand_msg.actual.positions
        #print("arm angle : {}".format(arm_data))
        obj_data = [obj_pos.x, obj_pos.y, obj_pos.z, obj_ori.x, obj_ori.y, obj_ori.z, obj_ori.w]
        #print("obj position: {}".format(obj_data))

        data_list = [int(time_msg.data)] + list(arm_data) + obj_data
        #print(data_list)
        self.data.append(data_list)



    def talker(self, sec):
        pub = rospy.Publisher('/my_timer', String, queue_size=10)
        time.sleep(0.4)
        r = rospy.Rate(FPS)
        count = 0

        while count < sec*FPS:
            str_co = str(count)
            #rospy.loginfo(str_co)
            pub.publish(str_co)
            #count = (count + 1) % FPS
            count = count + 1
            if count % (10*FPS) == 0:
                print("now {}".format(count/FPS))
            r.sleep()

        time.sleep(0.3)

        #import pprint
        #pprint.pprint(sorted(self.data))
        #print(np.array(self.data).shape)
        #time.sleep(0.3)


    def save_data(self,name):
        np.save(name, np.array(sorted(self.data)))





if __name__ == '__main__':
    if len(sys.argv) < 3:
        sec = 10
    else:
        sec = int(sys.argv[2])

    try:
        syn = dataSync()
        syn.talker(sec)
        syn.save_data(sys.argv[1])
        #rospy.spin()
    except:
        rospy.signal_shutdown("Quit")
        rospy.spin()
        #sys.exit()










