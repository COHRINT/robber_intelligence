#!/usr/bin/env python

import yaml
import random
import math
import tf
import rospy
import geometry_msgs
import nav_msgs
import move_base_msgs
import actionlib
import actionlib_msgs.msg as act_msgs # import *
import geometry_msgs.msg as geo_msgs # geo_msgs.Pose, geo_msgs.PoseWithCovarianceStamped, geo_msgs.Point, geo_msgs.Quaternion, geo_msgs.Twist, geo_msgs.Vector3
import move_base_msgs.msg as mov_msgs # import mov_msgs.MoveBaseAction, mov_msgs.MoveBaseGoal

def mapPub():
    # 2d Nav Goal
    # ask user for a goal if needed, otherwise automatically set by endpoint
    rospy.init_node('mapPub')
    rate = rospy.Rate(10)

    goal = mov_msgs.MoveBaseGoal()
    goal.target_pose.header.seq = 0
    goal.target_pose.header.stamp = 0
    goal.target_pose.header.frame_id = ""

    curfilePath = os.path.abspath(__file__)
    curDir = os.path.abspath(os.path.join(curfilePath, os.pardir))
    parentDir = os.path.abspath(os.path.join(curDir, os.pardir))
    mapInfo = parentDir + '/models/map2.yaml'
    with open(mapInfo, 'r') as stream:
        try:
            yamled = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    # deletes info
    del yamled['info']
    # makes list of location names
    # gets locations of each object, attaches them to name
    objDict = yamled.values()
    objLocations = {}
    for item in objDict:
        itemName = item['name']
        if itemName[0:4] != "wall":
            x_loc = item['centroid_x'] + (item['width']/2 + .6) * math.cos(math.radians(item['orientation']))
            y_loc = item['centroid_y'] + (item['length']/2 + .6) * math.sin(math.radians(item['orientation']))
            quat = tf.transformations.quaternion_from_euler(0, 0, item['orientation']-180)
            itemLoc = geo_msgs.Pose(geo_msgs.Point(x_loc, y_loc, 0), geo_msgs.Quaternion(quat[0],quat[1],quat[2],quat[3]))
            objLocations[itemName] = itemLoc
    vertexes = objLocations.values()
    vertexKeys = objLocations.keys()

    status = ['PENDING', 'ACTIVE', 'PREEMPTED',
        'SUCCEEDED', 'ABORTED', 'REJECTED',
        'PREEMPTING', 'RECALLING', 'RECALLED',
        'LOST']

    mover_base = actionlib.SimpleActionClient("zhora/move_base", mov_msgs.MoveBaseAction)
    mover_base.wait_for_server(rospy.Duration(5))

    # Keeping track
    n_locations = len(vertexes)
    i = 0

    rospy.loginfo("Starting Square Movement VROOM VROOM")

    # Go through the series of locations indefinitely
    while not rospy.is_shutdown():
        # Get random waypoint
        i = random.randint(0, n_locations-1)
        location = vertexes[i]
        rospy.loginfo("Going to " + vertexKeys[i])

        # Set up goal
        goal = mov_msgs.MoveBaseGoal()
        goal.target_pose.pose = location
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo(goal)

        # Start the robot toward the next location
        mover_base.send_goal(goal)
        rospy.loginfo("goal sent")

        # Allow 2 minutes to get there
        mover_base.wait_for_result(rospy.Duration(120))

        # Check status of movement
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


if __name__ == '__main__':
    try:
        mapPub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
