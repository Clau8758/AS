#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil
from std_msgs.msg import String
from gazebo_msgs.srv import GetModelState

def jointStatesCallback(msg):
  global currentJointState
  currentJointState = msg

def closeGripper():
  #rospy.init_node('gripperstateclose')
 
  # Setup subscriber
  #rospy.Subscriber("/joint_states", JointState, jointStatesCallback)
 
  pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
 
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  print 'Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  tmp = 0.7
  #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    pub.publish(currentJointState)
    print 'Published!'
    rate.sleep()
 
  print 'end!'

def openGripper():

  # Setup subscriber
  #rospy.Subscriber("/joint_states", JointState, jointStatesCallback)

  #rospy.init_node('gripperstateopen')
  pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
 
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  print 'Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  tmp = 0.005
  #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    pub.publish(currentJointState)
    print 'Published!'
    rate.sleep()
 
  print 'end!'

#Class defined to contain the name of the object in the scene and a relative fram
class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

#Class containing a list of block objects and relative objects - the robot_base
class container:
    _blockListDict = {
        'block_a': Block('cube0', 'robot_base'),
        'block_b': Block('cube1', 'robot_base'),
        'block_c': Block('cube2', 'robot_base'),
        'block_d': Block('cube3', 'robot_base'),
        'block_e': Block('cube4', 'robot_base'),
        'block_f': Block('cube5', 'robot_base'),
        'block_g': Block('cube6', 'robot_base'),
        'block_h': Block('cube7', 'robot_base'),
        'block_i': Block('bucket', 'ground_plane')
    }