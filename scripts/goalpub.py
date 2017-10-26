#!/usr/bin/env python

import rospy
import geometry_msgs
import nav_msgs
import move_base_msgs
import actionlib
import actionlib_msgs.msg as act_msgs # import *
import geometry_msgs.msg as geo_msgs # geo_msgs.Pose, geo_msgs.PoseWithCovarianceStamped, geo_msgs.Point, geo_msgs.Quaternion, geo_msgs.Twist, geo_msgs.Vector3
import move_base_msgs.msg as mov_msgs # import mov_msgs.MoveBaseAction, mov_msgs.MoveBaseGoal

def goalPub():
    # 2d Nav Goal
    # ask user for a goal if needed, otherwise automatically set by endpoint
    rospy.init_node('goalPub')
    navGoalPub = rospy.Publisher('move_base/goal', mov_msgs.MoveBaseGoal, queue_size=10) # could be /move_base/current_goal
    rate = rospy.Rate(10)

    goal = mov_msgs.MoveBaseGoal()
    goal.target_pose.header.seq = 0
    goal.target_pose.header.stamp = 0
    goal.target_pose.header.frame_id = ""

    vertexes = [geo_msgs.Pose(geo_msgs.Point(-1,1,0), geo_msgs.Quaternion(0,0,0,1)), geo_msgs.Pose(geo_msgs.Point(1,-1,0), geo_msgs.Quaternion(0,0,0,1)),
        geo_msgs.Pose(geo_msgs.Point(1,1,0), geo_msgs.Quaternion(0, 0,0,1)), geo_msgs.Pose(geo_msgs.Point(-1,-1,0), geo_msgs.Quaternion(0,0,0,1))]
    vertexDict = {'corner1':geo_msgs.Pose(geo_msgs.Point(1,1,0), geo_msgs.Quaternion(0,0,0,1)), 'corner2':geo_msgs.Pose(geo_msgs.Point(1,-1,0), geo_msgs.Quaternion(0,0,0,1)),
        'corner3':geo_msgs.Pose(geo_msgs.Point(-1,-1,0), geo_msgs.Quaternion(0,0,0,1)), 'corner4':geo_msgs.Pose(geo_msgs.Point(-1,1,0), geo_msgs.Quaternion(0,0,0,1))}
    vertexKeys = vertexDict.keys()
    status = ['PENDING', 'ACTIVE', 'PREEMPTED',
        'SUCCEEDED', 'ABORTED', 'REJECTED',
        'PREEMPTING', 'RECALLING', 'RECALLED',
        'LOST']
    # STEVE, HE KNEW FROM THE BEGINNING, WHAT IS THE COORDINATE SYSTEM?
    # is it bad coding practice to create classes for ros nodes instead of functions in python? I usually see only functions in the tuturial but cant get around
    # rospy subscriber callback function.

    #origin = geo_msgs.Pose(geo_msgs.Point(0,0,0), geo_msgs.Quaternion(0,0,0,0))

    mover_base = actionlib.SimpleActionClient("deckard/move_base", mov_msgs.MoveBaseAction)

    mover_base.wait_for_server(rospy.Duration(5))

    #rospy.on_shutdown(mover_base.cancel_goal())

    # Keeping track of things
    n_locations = len(vertexes)
    i = 0
    location = ""
    last_location = ""

    zero_covariance = [[0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0]]
    origin = geo_msgs.PoseWithCovarianceStamped(geo_msgs.Pose(geo_msgs.Point(0,0,0), geo_msgs.Quaternion(0,0,0,1)), zero_covariance)
    initialpose = origin
    # Get the initial pose from the user
    # initialpose = geo_msgs.PoseWithCovarianceStamped()
    # rospy.loginfo("*** Click the 2D geo_msgs.Pose Estimate button in RViz to set the robot's initial pose...")
    # rospy.wait_for_message('initialpose', geo_msgs.PoseWithCovarianceStamped)
    # rospy.Subscriber('initialpose', geo_msgs.PoseWithCovarianceStamped, update_initial_pose)

    rospy.loginfo("Starting Square Movement VROOM VROOM")

    # Go through the series of locations indefinitely
    while not rospy.is_shutdown():
        i += 1
        if i == n_locations:
            i = 0
        location = vertexes[i]
        rospy.loginfo(i)

        goal = mov_msgs.MoveBaseGoal()
        goal.target_pose.pose = location
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo(goal)

        # Start the robot toward the next location
        mover_base.send_goal(goal)
        rospy.loginfo("goal sent")

        # Allow 5 minutes to get there
        mover_base.wait_for_result(rospy.Duration(120))

        # Check for success or failure
        # if not finished_within_time:
        #     mover_base.cancel_goal()
        #     rospy.loginfo("FAILURE TO REACH DESTINATION... SHUTTING DOWN")
        #     mover_base.cancel_goal()
        #     #cmd_vel_pub.publish(geo_msgs.Twist(geo_msgs.Vector3(0,0,0),geo_msgs.Vector3(0,0,0)))
        #     # TODO: Shutdown function? or capabilities
        # else:
        state = mover_base.get_state()
        if state == 3: #SUCCESSFUL
            rospy.loginfo(i)
            rospy.loginfo("Just Reached " + vertexKeys[i])
        else:
          rospy.loginfo("Goal failed")
          rospy.loginfo(status[state])
          mover_base.cancel_goal()
          break
        rospy.sleep(1)

    # print("Position: ")
    # goal.target_pose.pose.position.x = input("x: ")
    # goal.target_pose.pose.position.y = input("y: ")
    # goal.target_pose.pose.position.z = 0
    #
    # print("geo_msgs.Quaternion: ")
    # goal.target_pose.pose.orientation.x = input("x: ")
    # goal.target_pose.pose.orientation.y = input("y: ")
    # goal.target_pose.pose.orientation.z = 0
    # goal.target_pose.pose.orientation.w = 0
    #
    # while not rospy.is_shutdown():
    #     if goal:
    #         rospy.loginfo(goal)
    #         navGoalPub.publish(goal)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        goalPub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
