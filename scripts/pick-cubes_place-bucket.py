#!/usr/bin/python2

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
    gazebo_environment_snap = gazebo_environment
    goal = gazebo_environment_snap.pose[gazebo_environment_snap.name.index(name)]
    goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,-0.5*math.pi,0))
    goal.position.z += 0.17
    move_group.set_pose_target(goal)

    # trajectory = move_group.plan()
    # while trajectory.joint_trajectory.points == []: 
    #     print "Increasing z by 0.01"
    #     goal.position.z += 0.01
    #     print "new z-value %f" % goal.position.z
    #     move_group.set_pose_target(goal)
    #     trajectory = move_group.plan()


    # DEBUG: Only rotate
    #goal = move_group.get_current_pose().pose
    #goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.,-.5*math.pi,0.))
    print "Goal : %s" % goal
    if display_trajectory_publisher != 0 and robot != 0:
        print "Visualizing trajectory"
        plan = move_group.plan()
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        print "Publishing trajectory"
        display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(2.)
    print "Moving to goal"
    move_group.go(wait=True)

def place_object_in_scene(scene, move_group, name):
    # Code for placing an object in a scene
    print "hello world"

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
                scene.add_mesh(name, p, '/home/pat/catkin_ws/src/hello_ros/urdf/bucket.dae')
            if "cube" in name:
                p.pose = gazebo_environment_snap.pose[index]
                print "adding %s at \n%s" % (name , p.pose)
                scene.add_box(name, p, (0.05,0.05,0.05))
        pick_object_in_scene(scene, group, "cube0", robot, display_trajectory_publisher)

    except rospy.ROSInterruptException:
        pass
        
