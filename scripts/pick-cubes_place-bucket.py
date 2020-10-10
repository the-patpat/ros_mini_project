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
from lecture_5_4_open_gripper import open_gripper
from lecture_5_4_close_gripper import close_gripper

def subscribe_gazebo_objects():
    rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, refresh_environment)

def refresh_environment(msg):
    # TODO maybe change this to a parameter approach
    global gazebo_environment
    gazebo_environment = msg
    z_positions = []
    for index, name in enumerate(msg.name):
        if "cube" in name:
            z_positions.append(msg.pose[index].position.z)
    with open('cube_pos', 'ab+') as f:
        for position in z_positions:
            f.write('%f;' % position)
        f.write('\n')
        f.close()

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
    print "[%f]pick_object_in_scene: Stopping residual movement to start picking process" % rospy.get_time()
    gazebo_environment_snap = gazebo_environment
    goal = group.get_current_pose().pose
    move_group.set_pose_target(goal)
    move_group.go(wait=True)

    # Open gripper
    print "[%f]pick_object_in_scene: Opening gripper in preparation for approaching %s" % (rospy.get_time(), name)
    open_gripper()
    goal = group.get_current_pose().pose
    move_group.set_pose_target(goal)
    move_group.go(wait=True)
    move_group.stop()
    rospy.sleep(2.)
   
    goal = gazebo_environment_snap.pose[gazebo_environment_snap.name.index(name)]
    
    print "[%f]pick_object_in_scene: Got position x:%f y:%f z:%f as %s position" % (rospy.get_time(),goal.position.x, goal.position.y, goal.position.z, name)
        
    # Move above the cube with gripper oriented towards the table top
    goal.position.z += 0.30 
    goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,-0.5*math.pi,0))
    print "[%f]pick_object_in_scene: Position setpoint above %s: x:%f y:%f z:%f" % (rospy.get_time(), name, goal.position.x, goal.position.y, goal.position.z)
    move_group.set_pose_target(goal)
    

    # Open gripper
    print "[%f]pick_object_in_scene: Opening gripper in preparation for approaching %s" % (rospy.get_time(), name)
    
    # Visualizing the trajectory in Rviz
    if display_trajectory_publisher != 0 and robot != 0:
        #print "Visualizing trajectory"
        plan = move_group.plan()
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        #print "Publishing trajectory"
        display_trajectory_publisher.publish(display_trajectory)
        rospy.sleep(2.)

    # Moving to the goal (above the cube) 
    #print "Moving to goal"
    print "[%f]pick_object_in_scene: Trying to execute movement to setpoint" % (rospy.get_time())
    if move_group.go(wait=True):
        print "[%f]pick_object_in_scene: Movement executed successfully" %(rospy.get_time()) 
    else:
        print "[%f]pick_object_in_scene: Movement execution failed, continuing with next object" %(rospy.get_time()) 
        return False 
    
  

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

    print "[%f]pick_object_in_scene: Planning cartesian trajectory to approach %s with desired movement from x1:%f -> x2:%f - y1:%f -> y2:%f - z1:%f -> z2:%f" % (
        rospy.get_time(), name, waypoints[0].position.x, waypoints[1].position.x, 
        waypoints[0].position.y, waypoints[1].position.y, waypoints[0].position.z, waypoints[1].position.z)
    #print "The waypoints are: %s" %waypoints

    # Compute the cartesian path and execute it
    (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
    ##print plan1
    print "[%f]pick_object_in_scene: Trying to execute planned trajectory" % (rospy.get_time())
    if group.execute(plan1, wait=True):
        print "[%f]pick_object_in_scene: Cartesian Movement successful" %(rospy.get_time()) 
    else:
        print "[%f]pick_object_in_scene: Cartesian Movement failed, continuing with next cube" %(rospy.get_time()) 
        return False

    #Close the gripper and wait a few seconds

    print "[%f]pick_object_in_scene: Closing gripper to pick up %s" %(rospy.get_time(), name)
    close_gripper()
    goal = group.get_current_pose().pose
    move_group.set_pose_target(goal)
    move_group.go(wait=True)
    move_group.stop()
    rospy.sleep(2.)
    


    waypoints = []
    # Move back to where we came from
    waypoints.append(group.get_current_pose().pose)
    waypoints.append(group.get_current_pose().pose)
    waypoints[1].position.z += 0.3

    #print "Move back to where we came from: %s" % waypoints

    print "[%f]pick_object_in_scene: Planning cartesian trajectory to lift cube up %s with desired movement from x1:%f -> x2:%f - y1:%f -> y2:%f - z1:%f -> z2:%f" % (
        rospy.get_time(), name, waypoints[0].position.x, waypoints[1].position.x,
        waypoints[0].position.y, waypoints[1].position.y, waypoints[0].position.z, waypoints[1].position.z)        
    # Compute the cartesian path and execute it
    (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
    print "[%f]pick_object_in_scene: Trying to execute planned trajectory" % (rospy.get_time())
    ##print plan1
    if group.execute(plan1, wait=True): 
        print "[%f]pick_object_in_scene: Cartesian Movement successful" %(rospy.get_time()) 
    else:
        print "[%f]pick_object_in_scene: Cartesian Movement failed, continuing with next cube" %(rospy.get_time())
        return False
    return True

def place_object_in_scene(scene, move_group, name, robot = 0, display_trajectory_publisher=0):
    # Code for placing an object in a scene
    print "[%f]place_object_in_scene: Stopping residual movement to start picking process" % rospy.get_time()
    move_group.stop()
    gazebo_environment_snap = gazebo_environment
    goal = group.get_current_pose().pose
    move_group.set_pose_target(goal)
    move_group.go(wait=True)

    
    # Move above the bucket
    goal = gazebo_environment_snap.pose[gazebo_environment_snap.name.index(name)]
    goal.position.z += 0.45
    goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,-0.5*math.pi,0))
    move_group.set_pose_target(goal)
    print "[%f]place_object_in_scene: Position setpoint above %s: x:%f y:%f z:%f" % (rospy.get_time(), name, goal.position.x, goal.position.y, goal.position.z)
    print "[%f]place_object_in_scene: Trying to execute movement to setpoint" % (rospy.get_time())

    if move_group.go(wait=True):
        print "[%f]place_object_in_scene: Movement executed successfully" %(rospy.get_time()) 
    else:
        print "[%f]place_object_in_scene: Movement execution failed, continuing with next object" %(rospy.get_time()) 
        return False 

    # Open the gripper
    print "[%f]place_object_in_scene: Opening gripper to place cube in bucket" % rospy.get_time()
    open_gripper()

    #Move to current position to stop residual movement
    move_group.stop()
    goal = move_group.get_current_pose().pose
    move_group.set_pose_target(goal)
    move_group.go(wait=True)
    return True


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
        #print "Sleeping for 5 Seconds"
        rospy.sleep(5)
        gazebo_environment_snap = gazebo_environment
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        for index, name in enumerate(gazebo_environment_snap.name):
            if "bucket" in name:
                p.pose = gazebo_environment_snap.pose[index]
                #print "adding %s at \n%s" % (name, p.pose)
                scene.add_box(name, p, (0.2,0.2,0.2))
            if "cube" in name:
                p.pose = gazebo_environment_snap.pose[index]
                #print "adding %s at \n%s" % (name , p.pose)
                scene.add_box(name, p, (0.05,0.05,0.05))
        
        failed_names = []
        for index, name in enumerate(gazebo_environment_snap.name):
            if "cube" in name:
                if not pick_object_in_scene(scene, group, name, robot, display_trajectory_publisher):
                    #Could not pick up object
                    failed_names.append(name)
                    open_gripper()
                    group.stop()
                else:
                    if not place_object_in_scene(scene, group, "bucket", robot, display_trajectory_publisher):
                        failed_names.append(name)
                        open_gripper()
                        group.stop()

        for i in range(0,2):
            for name in failed_names:                    
                if not pick_object_in_scene(scene, group, name, robot, display_trajectory_publisher):
                    failed_names.append(name)
                    open_gripper()
                    group.stop()
                else:
                    if not place_object_in_scene(scene, group, "bucket", robot, display_trajectory_publisher):
                        open_gripper()
                        group.stop()
                    else:
                        failed_names.delete(name)

        group.stop()
    except rospy.ROSInterruptException:
        pass
    except moveit_commander.MoveItCommanderException as e:
        #print "Exception: %s" % e 
        pass
    
        
