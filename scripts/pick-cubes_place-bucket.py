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


    # Get current environment 
    print "[%f]pick_object_in_scene: Stopping residual movement to start picking process" % rospy.get_time()
    gazebo_environment_snap = gazebo_environment
    
 
    # to avoid a non-functioning gripper because of residual movement
    goal = group.get_current_pose().pose
    move_group.set_pose_target(goal)
    move_group.go(wait=True)

    # Open gripper
    print "[%f]pick_object_in_scene: Opening gripper in preparation for approaching %s" % (rospy.get_time(), name)
    open_gripper()
    goal = group.get_current_pose().pose
    move_group.set_pose_target(goal)
    move_group.go(wait=True)
    rospy.sleep(2.)
   
    # Set cube position as preliminary goal (z position will be added)
    goal = gazebo_environment_snap.pose[gazebo_environment_snap.name.index(name)]

    x0, y0 = 0.767, 0.056	
    xc, yc = goal.position.x, goal.position.y	
    rsq = 0.5
    if (xc-x0)**2+(yc-y0)**2 > rsq:	
        print "[%f]pick_object_in_scene: %s is out of reach" % (rospy.get_time(), name)	
        return {'success':False, 'reason':'OOR'}
    
    
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
    print "[%f]pick_object_in_scene: Trying to execute movement to setpoint" % (rospy.get_time())
    if move_group.go(wait=True):
        print "[%f]pick_object_in_scene: Movement executed successfully" %(rospy.get_time()) 
    else:
        print "[%f]pick_object_in_scene: Movement execution failed, continuing with next object" %(rospy.get_time()) 
        return {'success':False, 'reason':'MEF'} 
    
  

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

    # Compute the cartesian path and execute it
    attempts = 0
    fraction = 0
    while fraction < 0.85 and attempts <= 2:
        (plan1, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        print "[%f]pick_object_in_scene: Attempt %d has fraction %f" %(rospy.get_time(), attempts, fraction)
        attempts += 1
    if fraction < 0.85:
        print "[%f]pick_object_in_scene: All three attempts to compute a cartesian path failed. Skipping this cube for now"
        return {'success':False, 'reason':'CMEF'}
    print "[%f]pick_object_in_scene: Trying to execute planned trajectory, fraction is: %f" % (rospy.get_time(), fraction)

    if group.execute(plan1, wait=True):
        cur_pos = group.get_current_pose().pose
        print "[%f]pick_object_in_scene: Cartesian Movement successful, current x:%f y:%f z:%f" %(rospy.get_time(), cur_pos.position.x, cur_pos.position.y, cur_pos.position.z) 
    else:
        print "[%f]pick_object_in_scene: Cartesian Movement failed, continuing with next cube" %(rospy.get_time()) 
        return {'success':False, 'reason':'CMEF'}
  
    #Close the gripper and wait a few seconds

    print "[%f]pick_object_in_scene: Closing gripper to pick up %s" %(rospy.get_time(), name)
    close_gripper()
    goal = group.get_current_pose().pose
    move_group.set_pose_target(goal)
    move_group.go(wait=True)
    rospy.sleep(2.)

    return {'success':True, 'reason':''}

def place_object_in_scene(scene, move_group, name, robot = 0, display_trajectory_publisher=0, name_picked=""):
    # Code for placing an object in a scene
    print "[%f]place_object_in_scene: Stopping residual movement to start picking process" % rospy.get_time()
    gazebo_environment_snap = gazebo_environment
    goal = group.get_current_pose().pose
    move_group.set_pose_target(goal)
    move_group.go(wait=True)
    rospy.sleep(2.)

    
    # Move above the bucket
    goal = gazebo_environment_snap.pose[gazebo_environment_snap.name.index(name)]
    goal.position.z += 0.55
    goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0,-0.5*math.pi,0))
    move_group.set_pose_target(goal)
    print "[%f]place_object_in_scene: Position setpoint above %s: x:%f y:%f z:%f" % (rospy.get_time(), name, goal.position.x, goal.position.y, goal.position.z)
    print "[%f]place_object_in_scene: Trying to execute movement to setpoint" % (rospy.get_time())

    if move_group.go(wait=True):
        print "[%f]place_object_in_scene: Movement executed successfully" %(rospy.get_time()) 
    else:
        print "[%f]place_object_in_scene: Movement execution failed, continuing with next object" %(rospy.get_time()) 
        return {'success':False, 'reason':''} 
    rospy.sleep(2.)
    # Open the gripper
    print "[%f]place_object_in_scene: Opening gripper to place cube in bucket" % rospy.get_time()
    open_gripper()
    rospy.sleep(2.)

    #Move to current position to stop residual movement
    goal = move_group.get_current_pose().pose
    move_group.set_pose_target(goal)
    move_group.go(wait=True)
    rospy.sleep(2.)
    return True

if __name__ == '__main__':
    try:

        # Initializing our script with everything necessary, then sleep for 5 seconds
        robot, scene, group = move_group_initialize("Arm")        
        rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, refresh_environment)
        display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

        print "[%f]Sleeping for 5 seconds to let model_states be populated, then get positions for MoveIt collision objects" % rospy.get_time()
        rospy.sleep(5)
        gazebo_environment_snap = gazebo_environment


        # Add collision objects to the planning scene
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        for index, name in enumerate(gazebo_environment_snap.name):
            if "bucket" in name:
                p.pose = gazebo_environment_snap.pose[index]
                p.pose.position.z += 0.1
                # add_mesh added the bucket in the middle of nowhere
                scene.add_box(name, p, (0.2,0.2,0.2))
            if "cube" in name:
                p.pose = gazebo_environment_snap.pose[index]
                scene.add_box(name, p, (0.05,0.05,0.05))
        
        failed_names = []
        redo_names = []

        # Iterate in reverse direction over the objects
        for index in range(len(gazebo_environment_snap.name)):
            name = gazebo_environment_snap.name[-1-index]	
            if "cube" in name:
                rval = pick_object_in_scene(scene, group, name, robot, display_trajectory_publisher)
                if not rval['success']:
                    #Could not pick up object
                    if rval['reason'] == 'OOR':
                        #Object is out of reach
                        failed_names.append(name)
                    else:
                        # Other failure, might be because controller failed during execution
                        redo_names.append(name)
                else:
                    if not place_object_in_scene(scene, group, "bucket", robot, display_trajectory_publisher, name):
                        # Movement execution failed, drop and try again later
                        redo_names.append(name)
                        open_gripper()
                    else:
                        # Check if box is really in the bucket
                        gazebo_environment_snap = gazebo_environment
                        cube_pos = gazebo_environment_snap.pose[gazebo_environment_snap.name.index(name)]
                        bucket_pos = gazebo_environment_snap.pose[gazebo_environment_snap.name.index("bucket")]
                        if (cube_pos.position.x - bucket_pos.position.x)**2 + (cube_pos.position.y - bucket_pos.position.y)**2 > 0.14**2:
                            redo_names.append(name)
                        else:
                            print "[%f] Placed %s successfully in bucket" % (rospy.get_time(), name)
                        

        for i in range(3):
            for name in redo_names:                    
                rval = pick_object_in_scene(scene, group, name, robot, display_trajectory_publisher)
                if not rval['success']:
                    #Could not pick up object
                    if rval['reason'] == 'OOR':
                        #Object is out of reach
                        if name not in failed_names:
                            failed_names.append(name)
                else:
                    if not place_object_in_scene(scene, group, "bucket", robot, display_trajectory_publisher, name):
                        # Movement execution failed, drop and try again later
                        open_gripper()
                    else:
                        # Check if box is really in the bucket
                        gazebo_environment_snap = gazebo_environment
                        cube_pos = gazebo_environment_snap.pose[gazebo_environment_snap.name.index(name)]
                        bucket_pos = gazebo_environment_snap.pose[gazebo_environment_snap.name.index("bucket")]
                        if not (cube_pos.position.x - bucket_pos.position.x)**2 + (cube_pos.position.y - bucket_pos.position.y)**2 > 0.14**2:
                            print "[%f] Placed %s successfully in bucket" % (rospy.get_time(), name)
                            redo_names.remove(name)
        if len(failed_names) != 0:
            print "[%f]Cubes that have failed: %s" % (rospy.get_time(), failed_names)
    except rospy.ROSInterruptException:
        pass
    except moveit_commander.MoveItCommanderException as e:
        pass
    
        
