#coding:UTF-8
import argparse
import math
import random
import numpy
import sys
from time import sleep
import cv2
import copy
import numpy as np
import pickle
import timeout_decorator

import rospy
import roslib
import rospkg
from cv_bridge import CvBridge
import tf
from tf.transformations import *

import moveit_commander
import moveit_msgs.msg

from sensor_msgs.msg import Image, CompressedImage
#import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState

from std_msgs.msg import (
        UInt16,
        String,
        )

from geometry_msgs.msg import (
        Pose,
        Point,
        Quaternion,
        PoseStamped,
        )

def is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False


class UrManipulator(object):
    def __init__(self):
        # Publishers
        self._obj_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)
        self.done_pub = rospy.Publisher("is_done",String,queue_size=4)
        self.reset_touch_pub = rospy.Publisher("reset_touch_flag",String,queue_size=4)

        rospy.on_shutdown(self.clean_shutdown)

        # moveit func
        self.arm_group = moveit_commander.MoveGroupCommander("ur5_arm")
        self.hand_group = moveit_commander.MoveGroupCommander("hand")
        self.hand_joint_name = "robotiq_85_left_knuckle_joint"
        self.hand_pos = {"close": math.pi/6, "open": math.pi/12}

        # add obstacle(table) to moveit
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(2)
        self.scene.remove_world_object("table")
        box_pose = PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = 0.45
        box_pose.pose.position.y = -0.3
        box_pose.pose.position.z = -0.35
        box_pose.pose.orientation.w = 1.0
        self.scene.add_box("table", box_pose, size=(1, 1, 0.3))

        # flag for task
        self.is_done = False

        # param init
        self.cmd = "0"
        self.action_range = [2, 9, 12, 12] # open/close, wrist, y, x
        #self._reward = 0


    def clean_shutdown(self):
        print("\nExiting...")
        # something need~~

    def reset_env(self):
        # init param
        self.is_done = False
        self.cmd = "0"

        self.rm_object()
        # arm reset
        self.set_neutral()
        self.gripper_move("open")

        # object position reset (checkout num)
        self.obj_random_spawn()


    def obj_random_spawn(self,obj_name="obj_1"):
        obj_pos = [0,0,0,0,0,0,0]
        obj_pos[0] = 0.5 + random.uniform(-0.05,0.05)
        obj_pos[1] = -0.35 + random.uniform(-0.05,0.05)
        obj_pos[2] = 0.33

        object_angle = random.uniform(0,math.pi)
        obj_pos[5] = math.sin(object_angle/2)
        obj_pos[6] = math.cos(object_angle/2)
        
        self.replace_object(obj_pos, obj_name="obj_1")


    def replace_object(self, place, obj_name="obj_1"):
        pos = Point(x=place[0], y=place[1], z=place[2]+0.01)
        ori = Quaternion(x=place[3], y=place[4], z=place[5], w=place[6])
        # set position
        modelstate = ModelState()
        modelstate.model_name = obj_name
        modelstate.reference_frame = "world"
        modelstate.pose = Pose(position=pos, orientation=ori)
        req = self._obj_state(modelstate)

    def rm_object(self, obj_name="obj_1"):
        # move object brhind arm
        obj_pos = [0,0,0,0,0,0,0]
        obj_pos[0] = -0.2
        obj_pos[1] = 0.2
        obj_pos[2] = 0.1
        self.replace_object(obj_pos, obj_name="obj_1")


    def get_angles(self):
        joints = self.arm_group.get_current_joint_values()
        return joints

    def get_endpoint(self):
        position = self.arm_group.get_current_pose().pose
        return position

    # goto action 0 position
    def set_neutral(self):
        print("go to neutral position...")
        joint_goal = [-0.7921126790274347, -0.932559083781797, 2.216348689347363, -2.854560900878817, -1.5707964956997724, -0.7921132469544414]
        self.arm_group.go(joint_goal, wait=True)
        print("neutral position now")

    # move arm from endpoint
    def make_ik_move(self,pose):
        self.arm_group.set_pose_target(pose)
        self.arm_group.go()
        self.arm_group.clear_pose_targets()

    # move from waypoint / current position -> wrist moved -> middle waypoints -> target_position
    def make_way_move(self, pose):
        target_ep = pose
        waypoints = []
        now_ep = self.get_endpoint()
        diff_x = target_ep.position.x - now_ep.position.x
        diff_y = target_ep.position.y - now_ep.position.y
        diff_z = target_ep.position.z - now_ep.position.z
        distance = math.sqrt(diff_x**2 + diff_y**2)

        num = int(1+distance*100 // 2) # number of update
        ep = now_ep
        ep.orientation = Quaternion(x=target_ep.orientation.x, y=target_ep.orientation.y, z=target_ep.orientation.z, w=target_ep.orientation.w)
        waypoints.append(copy.deepcopy(ep))

        for i in range(num-1):
            ep.position.x += diff_x/num
            ep.position.y += diff_y/num
            ep.position.z += diff_z/num
            waypoints.append(copy.deepcopy(ep))

        ep.position = Point(x=target_ep.position.x, y=target_ep.position.y,z=target_ep.position.z)
        ep.orientation = Quaternion(x=target_ep.orientation.x, y=target_ep.orientation.y, z=target_ep.orientation.z, w=target_ep.orientation.w)
        waypoints.append(copy.deepcopy(ep))
        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0, True)
        self.arm_group.execute(plan)


    # move gripper / status : "open" or "close"
    def gripper_move(self,status):
        self.hand_group.go({self.hand_joint_name: self.hand_pos[status]}, wait=True)

    def action(self, cmd):
        # decode cmd
        if type(cmd) == str:
            q = int(cmd)
        else:
            q = cmd
        target_point = [0 for _ in range(len(self.action_range))]
        for i in range(len(self.action_range)):
            q, mod = divmod(q, self.action_range[i])
            target_point[i] = mod

        # change decoded cmd to position
        target_x = 0.4 + (0.3/self.action_range[3]) * target_point[3]
        target_y = -0.25 - (0.3/self.action_range[2]) * target_point[2]
        wrist_angle = (math.pi/self.action_range[2]) * target_point[1]
        is_open = target_point[0]

        target_pose=Pose()
        target_pose.position = Point(x=target_x, y=target_y, z=-0.028)
        qu = quaternion_from_euler(wrist_angle, math.pi/2, 0) # default position (0, math.pi/2, 0)
        target_pose.orientation = Quaternion(x=qu[0], y=qu[1], z=qu[2], w=qu[3])
        #target_pose.orientation = Quaternion(x=0.0, y=0.7071, z=0.00, w=0.7071)
        self.make_way_move(target_pose)

        # move grippper to desired position
        if is_open:
            self.gripper_move("open")
        else:
            self.gripper_move("close")
        
        self.is_done = self.check_done()


    # if object fall off the table, return True else return False
    def check_done(self):
        obj_pos = self.get_obj_pos()
        if obj_pos.position.z < 0.25:
            return True
        else:
            return False


    def action_callback(self,data):
        if is_int(data.data):
            self.save_env_data(data.data)
            self.cmd = data.data
            self.action(self.cmd)
            self.done_pub.publish(str(self.is_done)) # publish "True"/"False"
        elif data.data == "recover":
            self.recover_env()
            return 0
        elif data.data == "reset":
            self.reset_env()
            self.done_pub.publish(str(self.is_done)) # publish "True"/"False"
        else:
            print("unexcepted value!")

    def listener(self):
        rospy.Subscriber("actions", String, self.action_callback)
        rospy.spin()

    def get_img(self):
        for _ in range(2):
            img_data = None
            while img_data is None:
                img_data = rospy.wait_for_message("/camera/image_raw/compressed", CompressedImage,timeout=None)

    def recover_env(self):
        self.reset_env()
        # load env_data
        env_data = np.load("saved_env.npz")
        before_action = env_data["before_action"][0]
        action = env_data["action"][0]
        obj_pos = env_data["obj_pos"]
        # set env from data
        self.action(before_action)
        self.replace_object(obj_pos)
        self.reset_touch_pub.publish("flag from recover_env func")
        # do action
        self.action(action)
        self.done_pub.publish(str(self.is_done)) # publish "True"/"False"

    def get_obj_pos(self,obj_name="obj_1"):
        try:
            get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            obj_pos = get_model(obj_name, "")
            obj_pos = obj_pos.pose
            return obj_pos
        except rospy.ServiceException, e:
            rospy.loginfo("Get Model service call failed: {0}".format(e))

    def save_env_data(self,action):
        obj_pos = self.get_obj_pos()
        obj_list=[obj_pos.position.x, obj_pos.position.y, obj_pos.position.z, obj_pos.orientation.x, obj_pos.orientation.y, obj_pos.orientation.z, obj_pos.orientation.w]
        np.savez("saved_env.npz", before_action=np.array([int(self.cmd)]), action=np.array([int(action)]), obj_pos=np.array(obj_list))




if __name__ == '__main__':
    print("Initializing node... ")
    rospy.init_node("ur_sample")

    ur_manipulator = UrManipulator()
    print("mainpulator ready")
    #print(ur_manipulator.get_angles())
    #print(ur_manipulator.get_endpoint())

    #pose = ur_manipulator.get_endpoint()
    #pose.position.y -= 0.2
    #ur_manipulator.make_ik_move(pose)

    ur_manipulator.reset_env()
    #ur_manipulator.save_env_data("800")

    before = rospy.get_time()
    for i in range(30):
        #action = i
        action = np.argmax(np.random.rand(2592))
        print(action)
        ur_manipulator.action(action)
        if ur_manipulator.is_done:
            ur_manipulator.reset_env()

        if (i+1) % 25 == 0:
            print("!! {} step end now !!".format(i+1))


    after = rospy.get_time()
    print("action time: {}".format(after-before))

    #print("recovering...")
    #ur_manipulator.recover_env()

    print("reset env")
    ur_manipulator.reset_env()

    #ur_manipulator.action(0)

