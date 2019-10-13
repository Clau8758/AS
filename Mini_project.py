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
from Motionplanner import *
from Block_classes import *
#from Block_classes import container 


tuto = container()

if __name__ == '__main__':
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    cube_coord=[]
    for block in tuto._blockListDict.itervalues():
      i=0;
      blockName = str(block._name)
      resp_coordinates = (model_coordinates(blockName, block._relative_entity_name))
      if (resp_coordinates.success):
          cube_coord.append(model_coordinates(blockName, block._relative_entity_name))
          i=i+1;
try:
      move_group_python_mini_project()
except rospy.ROSInterruptException:
      pass


