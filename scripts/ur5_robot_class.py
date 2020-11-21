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

import moveit_commander
import moveit_msgs.msg

from sensor_msgs.msg import Image, CompressedImage
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
        self.hand_pos = {"close": math.pi/9, "open": 0.0}


        # flag for task
        # nothing

        # param init
        #self.cmd = 0
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
        obj_pos[0] = 0.3 + random.uniform(-0.1,0.1)
        obj_pos[1] = -0.3 + random.uniform(-0.1,0.1)
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
        pass





if __name__ == '__main__':
    print("Initializing node... ")
    rospy.init_node("ur_sample")

    ur_manipulator = UrManipulator()
    ur_manipulator.reset_env()
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

    import geometry_msgs.msg
    scene = moveit_commander.PlanningSceneInterface()
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "test_obstacle"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    timeout = 10
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
          
      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()
                
      # Test if we are in the expected state
      if (True == is_attached) and (True == is_known):
        break
           
      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      print(is_attached)
      seconds = rospy.get_time()
                              

    print(seconds - start)
    print("hello")


