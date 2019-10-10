#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
import motionplanner.py
import Block_classes.py


currentJointState = JointState()


if __name__ == '__main__':
    tuto = container()
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    cube_coord=[]
    for block in tuto._blockListDict.itervalues():
      i=0;
      blockName = str(block._name)
      resp_coordinates = (model_coordinates(blockName, block._relative_entity_name))
      if (resp_coordinates.success):
          cube_coord.append(model_coordinates(blockName, block._relative_entity_name))
          print '\n'
          print 'Status.success = ', resp_coordinates.success
          print(blockName)
          print("Cube " + str(block._name))
          print("Valeur de X : " + str(resp_coordinates.pose.position.x))
          print("Quaternion y : " + str(resp_coordinates.pose.position.y))
          print("Quaternion z : " + str(resp_coordinates.pose.position.z))
          print("Valeur de X : " + str(resp_coordinates.pose.orientation.x))
          print("Quaternion y : " + str(resp_coordinates.pose.orientation.y))
          print("Quaternion z : " + str(resp_coordinates.pose.orientation.z))
          print("Quaternion w : " + str(resp_coordinates.pose.orientation.w))
          print("Cube " + str(block._name))
          i=i+1;
    try:
      move_group_python_mini_project()
    except rospy.ROSInterruptException:
      pass


