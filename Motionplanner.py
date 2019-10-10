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

def move_group_python_mini_project():
  ## BEGIN MOTION PLANNING
  ## First initialize moveit_commander and rospy.
  print "============ Starting motion planning setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_mini_project',
                  anonymous=True)
 
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm")
 
  #p=geometry_msgs.PoseStamped()
  #p.header.frame_id = robot.get_planning_frame
  #p.pose.position.x=cube_coord[0].pose.position.x
  #p.pose.position.y=cube_coord[0].pose.position.y
  #p.pose.position.z=cube_coord[0].pose.position.z
  #scene.add_box("box",p,(0.25,0.25,0.3))
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
 
  print "============ Starting motion planning "
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()
  ## We can also print the name of the end-effector link for this group
  print "============ End effector frame: %s" % group.get_end_effector_link()
  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()
  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"

  ## Let's setup the planner
  group.set_planning_time(0.0)
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_tolerance(0.01)
  group.set_goal_joint_tolerance(0.05)
  group.set_num_planning_attempts(100)   
 
  print "============ Generating starting plan"
    
  pose_goal = group.get_current_pose().pose
  waypoints = []
  pose_goal.orientation = 			geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.,  -math.pi/2.  ,0.))
  waypoints.append(pose_goal)
  pose_goal.position.x =0.1
  pose_goal.position.y =0.15
  pose_goal.position.z =1.3
  print pose_goal

#Create waypoints
  waypoints.append(pose_goal)

#createcartesian  plan
  (plan1, fraction) = group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
#plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)


  print "============ Waiting while RVIZ displays plan1..."



## You can ask RVIZ to visualize a plan (aka trajectory) for you.
  print "============ Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);
  print "============ Waiting while plan1 is visualized (again)..."
  



## Moving to a pose goal
  group.execute(plan1,wait=True)
  rospy.sleep(10.)
 ######################################################################################################### Planning to a Pose goal - Movement to top of object
  for i in range(1,len(cube_coord)):
	
        print "============ Generating plan" , i ,"."
    
        pose_goal = group.get_current_pose().pose
        waypoints = []
        pose_goal.orientation = 			geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.,  -math.pi/2.  ,0.))
        waypoints.append(pose_goal)
        pose_goal.position.x =cube_coord[i].pose.position.x
        pose_goal.position.y =cube_coord[i].pose.position.y
        pose_goal.position.z =cube_coord[i].pose.position.z+0.3
        print pose_goal
    
        #Create waypoints
        waypoints.append(pose_goal)
    
        #createcartesian  plan
        (plan1, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
    
    
        print "============ Waiting while RVIZ displays plan1..."
     
    
    
        ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
        print "============ Visualizing plan1"
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        display_trajectory_publisher.publish(display_trajectory);
        print "============ Waiting while plan1 is visualized (again)..."
     
    

        
        ## Moving to a pose goal
        group.execute(plan1,wait=True)
        rospy.sleep(4.)
        
        ####### Go down to cube
        
        pose_goal = group.get_current_pose().pose
        waypoints = []
        pose_goal.orientation = 			geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.,  -math.pi/2.  ,0.))
        waypoints.append(pose_goal)
        pose_goal.position.x =cube_coord[i].pose.position.x
        pose_goal.position.y =cube_coord[i].pose.position.y
        pose_goal.position.z =cube_coord[i].pose.position.z+0.18
        print pose_goal
    
        #Create waypoints
        waypoints.append(pose_goal)
    
        #createcartesian  plan
        (plan1, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
    
    
        print "============ Waiting while RVIZ displays plan1..."
    
    
    
        ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
        print "============ Visualizing plan1"
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        display_trajectory_publisher.publish(display_trajectory);
        print "============ Waiting while plan1 is visualized (again)..."
        
    

        
        ## Moving to a pose goal
        group.execute(plan1,wait=True)
        rospy.sleep(4.)
        closeGripper()
        rospy.sleep(3.)
        
        ###### GO up from cube
        
        pose_goal = group.get_current_pose().pose
        waypoints = []
        pose_goal.orientation = 			geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.,  -math.pi/2.  ,0.))
        waypoints.append(pose_goal)
        pose_goal.position.x =cube_coord[i].pose.position.x
        pose_goal.position.y =cube_coord[i].pose.position.y
        pose_goal.position.z =cube_coord[i].pose.position.z+0.4
        print pose_goal
    
        #Create waypoints
        waypoints.append(pose_goal)
    
        #createcartesian  plan
        (plan1, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
    
    
        print "============ Waiting while RVIZ displays plan1..."
       
    
    
        ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
        print "============ Visualizing plan1"
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        display_trajectory_publisher.publish(display_trajectory);
        print "============ Waiting while plan1 is visualized (again)..."
    
    

        
        ## Moving to a pose goal
        group.execute(plan1,wait=True)
        rospy.sleep(4.)
        
        
        
        
        

        ################################################################################################### second movement - We move towards the bucket which has the same coordinate every time
        pose_goal2 = group.get_current_pose().pose
        waypoints2 = []
        pose_goal2.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.,  -math.pi/2.  , 0.))
        waypoints.append(pose_goal)
        pose_goal2.position.x = cube_coord[0].pose.position.x
        pose_goal2.position.y =cube_coord[0].pose.position.y
        pose_goal2.position.z =cube_coord[0].pose.position.z+0.4
        print pose_goal2
    
        #Create waypoints
        waypoints2.append(copy.deepcopy(pose_goal2))
    
        #createcartesian  plan
        (plan2, fraction) = group.compute_cartesian_path(
                                        waypoints2,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
     
    
        ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan2)
        display_trajectory_publisher.publish(display_trajectory);
    
   
    
        group.execute(plan2,wait=True)
        rospy.sleep(4.)
        openGripper()
        rospy.sleep(3)




  	## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

    ## End of motionplanning function
  print "============ STOPPING"
  R = rospy.Rate(10)
  while not rospy.is_shutdown():
    R.sleep()