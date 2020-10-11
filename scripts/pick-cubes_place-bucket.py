#!/usr/bin/env python

import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import gazebo_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
from std_msgs.msg import String
import math
from lecture_5_4_open_gripper import open_gripper
from lecture_5_4_close_gripper import close_gripper

def subscribe_gazebo_objects():
    rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, refresh_environment)

def refresh_environment(msg):
    # TODO maybe change this to a parameter approach
    global gazebo_environment
    gazebo_environment = msg

def move_group_initialize(name):
    # Initialize moveit_commander 
    moveit_commander.roscpp_initialize(sys.argv)
    # Initialize rospy node
    rospy.init_node('move_group_pick_place')
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander(name)
    group.set_goal_orientation_tolerance(0.01)
    group.set_goal_tolerance(0.01)
    group.set_goal_joint_tolerance(0.01)
    group.set_num_planning_attempts(100)
    return robot, scene, group

def pick_object_in_scene(scene, move_group, name, robot = 0, display_trajectory_publisher=0):
    # Code for picking a cube referenced by its name in the planning scene
    # First step, move there with a simple pose, no trajectory
    # hand limb frame has x-axis as approching axis

    # Get the current pose of the end effector and move there
    # to avoid a non-functioning gripper because of residual movement
    gazebo_environment_snap = gazebo_environment
    goal = group.get_current_pose().pose
    group.set_pose_target(goal)
    group.go(wait=True)
    rospy.sleep(5.)

    goal = gazebo_environment_snap.pose[gazebo_environment_snap.name.index(name)]

    x0, y0 = 0.767, 0.056
    xc, yc = goal.position.x, goal.position.y
    rsq = 0.5#(0.180-0.767)**2+(0.125-0.056)**2
    if (xc-x0)**2+(yc-y0)**2 > rsq:
	print "%s is out of reach" % name
	return False
    
    #print "Cube position before changing stuff %s" % goal
        
    # Move above the cube with gripper oriented towards the table top
    goal.position.z += 0.30 
    goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,-0.5*math.pi,0))
    group.set_pose_target(goal)
    
    # Print the goal to stdout
    #print "Position above the cube ready to approach the cube : %s" % goal

    # Open gripper
    open_gripper()
    rospy.sleep(5.)
    
    # Visualizing the trajectory in Rviz
    #if display_trajectory_publisher != 0 and robot != 0:
    #    print "Visualizing trajectory"
    #    plan = move_group.plan()
    #    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #    display_trajectory.trajectory_start = robot.get_current_state()
    #    display_trajectory.trajectory.append(plan)
    #    print "Publishing trajectory"
    #    display_trajectory_publisher.publish(display_trajectory)
    #    rospy.sleep(2.)

    # Moving to the goal (above the cube) 
    print "Moving to goal"
    group.go(wait=True)
    rospy.sleep(5.)

    # Move down to the cube with gripper opened and approach it with 17 cm above the cubes origin
    waypoints = []
    
    #Add current point to the trajectory as a starting point
    goal = group.get_current_pose().pose
    
    #print "Adding waypoint (current pose) %s" % goal
    waypoints.append(goal)

    #Add point 17 cm above cube's origin with gripper turned to the table top
    goal = gazebo_environment_snap.pose[gazebo_environment_snap.name.index(name)]
    goal.position.z -= 0.30
    goal.position.z += 0.17

    #print "Adding waypoint (gripping position) %s" % goal
    waypoints.append(goal)
    
    #print "The waypoints are: %s" %waypoints

    # Compute the cartesian path and execute it
    (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
    #print plan1
    rospy.sleep(5.)
    group.execute(plan1, wait=True)
    rospy.sleep(5.)

    #Close the gripper and wait a few seconds

    close_gripper()
    rospy.sleep(5.)

    goal = group.get_current_pose().pose
    group.set_pose_target(goal)
    print group.get_joint_value_target()
    group.go(wait=True)
    rospy.sleep(5.)
    

    #waypoints = []
    ## Move back to where we came from
    #waypoints.append(group.get_current_pose().pose)
    #waypoints.append(group.get_current_pose().pose)
    #waypoints[1].position.z += 0.3

    ##print "Move back to where we came from: %s" % waypoints

    ## Compute the cartesian path and execute it
    #(plan1, fraction) = group.compute_cartesian_path(
    #                                  waypoints,   # waypoints to follow
    #                                  0.01,        # eef_step
    #                                  0.0)         # jump_threshold
    ##print plan1
    #rospy.sleep(5.)
    #group.execute(plan1, wait=True)
    #rospy.sleep(5.)

    return True





def place_object_in_scene(scene, move_group, name, robot = 0, display_trajectory_publisher=0):
    # Code for placing an object in a scene
    gazebo_environment_snap = gazebo_environment
    goal = group.get_current_pose().pose
    group.set_pose_target(goal)
    group.go(wait=True)
    rospy.sleep(5.)

    
    # Move above the bucket
    goal = gazebo_environment_snap.pose[gazebo_environment_snap.name.index(name)]
    goal.position.z += 0.45
    goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,-0.5*math.pi,0))
    group.set_pose_target(goal)
    group.go(wait=True)
    rospy.sleep(5.)

    # Open the gripper
    open_gripper()
    rospy.sleep(5.)

    #Move to current position to stop residual movement
    goal = group.get_current_pose().pose
    group.set_pose_target(goal)
    group.go(wait=True)
    rospy.sleep(5.)


def add_object_to_planning_scene(scene, name, pose, size):
    # Add object to moveit planning scene (collision avoidance)
    print "hello world"



if __name__ == '__main__':
    try:
        robot, scene, group = move_group_initialize("Arm")        
        rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, refresh_environment)
        display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
        print "Sleeping for 5 Seconds"
        rospy.sleep(5)
        gazebo_environment_snap = gazebo_environment
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        for index, name in enumerate(gazebo_environment_snap.name):
            if "bucket" in name:
                p.pose = gazebo_environment_snap.pose[index]
                print "adding %s at \n%s" % (name, p.pose)
                scene.add_box(name, p, (0.2,0.2,0.3))
            if "cube" in name:
                p.pose = gazebo_environment_snap.pose[index]
                print "adding %s at \n%s" % (name , p.pose)
                scene.add_box(name, p, (0.05,0.05,0.05))
        
	#print gazebo_environment_snap.name
        for index in range(len(gazebo_environment_snap.name)):
	    name = gazebo_environment_snap.name[-1-index]
	    #print(name)
            if "cube" in name:
                if pick_object_in_scene(scene, group, name, robot, display_trajectory_publisher):
                    place_object_in_scene(scene, group, "bucket", robot, display_trajectory_publisher)

    except rospy.ROSInterruptException:
        pass
    except moveit_commander.MoveItCommanderException as e:
        print "Exception: %s" % e 
        pass
    group.stop()
        
