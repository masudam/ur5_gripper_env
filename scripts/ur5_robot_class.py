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



class UrManipulator(object):
    def __init__(self):
        # Publishers
        # must needed
        self._obj_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)


        # maybe need?
        #self.image_pub = rospy.Publisher("baxter_view",CompressedImage,queue_size=4) # fixed img publish
        #self.obj_pub = rospy.Publisher("obj_done",String,queue_size=4) # done signal publisher


        # control parameters
        self.bridge = CvBridge()

        # other
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
        box_pose.pose.position.x = 0.5
        box_pose.pose.position.y = -0.35
        box_pose.pose.position.z = -0.35
        box_pose.pose.orientation.w = 1.0
        self.scene.add_box("table", box_pose, size=(0.4, 0.4, 0.3))


        # flag for task
        # nothing

        # param init
        #self.cmd = 0
        self.action_range = [2, 9, 10, 10] # open/close, wrist, y, x
        self._reward = 0


    def clean_shutdown(self):
        print("\nExiting...")
        # something need~~

    def reset_env(self):
        # something need to reset ur5 position

        # object position reset (checkout num)
        self.obj_random_spawn()


    def obj_random_spawn(self,obj_name="obj_1"):
        obj_pos = [0,0,0,0,0,0,0]
        obj_pos[0] = 0.5 + random.uniform(-0.1,0.1)
        obj_pos[1] = -0.35 + random.uniform(-0.1,0.1)
        obj_pos[2] = 0.33

        object_angle = random.uniform(0,math.pi)
        obj_pos[5] = math.sin(object_angle/2)
        obj_pos[6] = math.cos(object_angle/2)
        
        self.replace_object(obj_pos, obj_name="obj_1")


    def replace_object(self, place, obj_name="obj_1"):
        pos = Point(x=place[0], y=place[1], z=place[2])
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

    def set_neutral(self):
        #pos = Point(x=place[0], y=place[1], z=place[2])
        #ori = Quaternion(x=place[3], y=place[4], z=place[5], w=place[6])
        pass

    # move arm from endpoint
    def make_ik_move(self,pose):
        self.arm_group.set_pose_target(pose)
        # plan and execute
        self.arm_group.go()
        self.arm_group.clear_pose_targets()

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
        target_x = 0.3 + (0.4/self.action_range[3]) * target_point[3]
        target_y = -0.15 - (0.4/self.action_range[2]) * target_point[2]
        wrist_angle = -math.pi/2 + (math.pi/self.action_range[2]) * target_point[1]
        is_open = target_point[0]

        target_pose=Pose()
        target_pose.position = Point(x=target_x, y=target_y, z=-0.029)
        qu = quaternion_from_euler(wrist_angle, math.pi/2, 0) # default (0, math.pi/2, 0)
        target_pose.orientation = Quaternion(x=qu[0], y=qu[1], z=qu[2], w=qu[3])
        #target_pose.orientation = Quaternion(x=0.0, y=0.7071, z=0.00, w=0.7071)
        print(target_pose)
        self.make_ik_move(target_pose)

        if is_open:
            self.gripper_move("open")
        else:
            self.gripper_move("close")



if __name__ == '__main__':
    print("Initializing node... ")
    rospy.init_node("ur_sample")

    ur_manipulator = UrManipulator()
    #ur_manipulator.reset_env()
    print(ur_manipulator.get_angles())
    print(ur_manipulator.get_endpoint())

    #pose = ur_manipulator.get_endpoint()
    #pose.position.y += 0.2
    #ur_manipulator.make_ik_move(pose)

    #pose = ur_manipulator.get_endpoint()
    #pose.position.y -= 0.2
    #ur_manipulator.make_ik_move(pose)

    #ur_manipulator.rm_object()
    ur_manipulator.gripper_move("close")
    ur_manipulator.gripper_move("open")
    ur_manipulator.gripper_move("close")

    for _ in range(3):
        action = np.argmax(np.random.rand(1800))
        print(action)
        ur_manipulator.action(action)

    print("hello")


