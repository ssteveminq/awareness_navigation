#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface 
from hsrb_interface import Robot
from hsrb_interface import geometry
import rospy
import sys
import roslib
import math
import tf
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Point
import moveit_commander
import trajectory_msgs.msg
import moveit_msgs
from copy import deepcopy
import moveit_msgs.msg
import rospy
import shape_msgs.msg
from tf.transformations import quaternion_from_euler, quaternion_multiply



# Move timeout[s]
_MOVE_TIMEOUT=60.0
# Grasp torque[Nm]
_GRASP_TORQUE=-0.01
# TF name of the mug
_DOOR_TF='door_handle'
# TF name of the gripper
_HAND_TF='hand_palm_link'
# TF name of the map
_MAP_TF='map'

# Grasp position from the mug
# _TRANSLATION_HANDLE_TO_GRASP = (
#             Point(0, -0.05, 0), Point(0.05, 0, 0), Point(0, 0.05, 0), Point(-0.05, 0, 0))


hand_up = geometry.pose(x=0.1)
# Posture to move the hand 0.5[m] back
hand_back = geometry.pose(z=-0.5)
# Location of the door
door_pos = (1.5, -0.2, 0.00)


class OpenDoorDemo(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)

        # self.robot=Robot()
        # self.listener =tf.TransformListener()
        # self.tts = self.robot.try_get('default_tts')
        self.reference_frame = "odom"
        #self.arm = moveit_commander.MoveGroupCommander("arm")
        # self.base = moveit_commander.MoveGroupCommander("base")
        # self.gripper = moveit_commander.MoveGroupCommander("gripper")
        # self.head = moveit_commander.MoveGroupCommander("head")
        self.whole_body = moveit_commander.MoveGroupCommander("whole_body")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.whole_body.allow_replanning(True)
        self.whole_body.set_planning_time(5)
        self.whole_body.set_pose_reference_frame(self.reference_frame)
        self.end_effector = self.whole_body.get_end_effector_link()
        rospy.sleep(1)

        # remove all objects
        self.scene.remove_attached_object(self.end_effector)
        self.scene.remove_world_object()
        rospy.sleep(1)

        # try:
        #     self.whole_body.move_to_neutral()
        # except:
        #     rospy.logerr('Fail move_to_neutral')

        # Greet
        # self.tts.say(u'Hi. My name is HSR. I will opend the door.')



    #  try:
    #     base.go(sofa_pos[0], sofa_pos[1], sofa_pos[2], _MOVE_TIMEOUT)
    # except:
    #     rospy.logerr('fail to move')
    #     sys.exit()
     #   moveit_commander.roscpp_initialize(sys.argv)
    
    
    def demo(self, wait=0.0):

        rospy.loginfo("step: move hand forward")
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "hand_palm_link"
        p.pose.position.x =  0.1
        p.pose.position.z = -0.2
        p.pose.orientation.w = 1
        self.whole_body.set_joint_value_target(p)
        self.whole_body.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

    # #whole_body.move_to_joint_positions({ 'head_pan_joint': -0.5399978289309297, 'head_tilt_joint': -1.0199961390659693})
    # #whole_body.move_to_joint_positions({'arm_flex_joint': -0.6645455020736355, 'arm_lift_joint': 0.583597213733165, 'arm_roll_joint': -0.21266819619457})
   
    # #rospy.init_node('tf_coke2')
    
    
    # #(trans,rot) = listener.lookupTransform('/base_link', '/door_handle', rospy.Time(0))
    # print "coke position:",trans
    

    # whole_body.end_effector_frame = u'hand_palm_link'
    
    # #whole_body.move_end_effector_pose( geometry.pose(x=trans[0],y=trans[1],z=trans[2]+0.3,ej=-1.57), 'base_link')
    
    # print "moving to the object"
    # #whole_body.move_end_effector_pose(geometry.pose(x=0.5,y=0.5,z=0.1+0.25), 'base_link')
    # whole_body.move_end_effector_pose(geometry.pose(ej=-1.57), 'hand_palm_link')
    # gripper.command(1.2)
    # whole_body.move_end_effector_pose(geometry.pose(z=0.20), 'hand_palm_link')
    # gripper.grasp(-0.01)
    # whole_body.move_end_effector_by_line((0, 0, -0.4), 0.1)
    # whole_body.move_to_neutral()
    


if __name__ == "__main__":
    rospy.init_node("moveit_demo", anonymous=True)
    utils = OpenDoorDemo(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
    utils.demo()

    # finalize
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
