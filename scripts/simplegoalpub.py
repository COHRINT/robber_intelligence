#!/usr/bin/env python

import rospy
import nav_msgs
import geometry_msgs.msg as geo_msgs
import move_base_msgs.msg as mov_msgs
import std_msgs

def simpleGoalPub():
    rospy.init_node('simpleGoalPub')
    navGoalPub = rospy.Publisher('roy/move_base_simple/goal', geo_msgs.PoseStamped, queue_size=10) # could be /move_base/current_goal

    vertexes = [geo_msgs.Pose(geo_msgs.Point(1,1,0), geo_msgs.Quaternion(0,0,0,1)), geo_msgs.Pose(geo_msgs.Point(1,-1,0), geo_msgs.Quaternion(0,0,0,1)),
        geo_msgs.Pose(geo_msgs.Point(-1,-1,0), geo_msgs.Quaternion(0,0,0,1)), geo_msgs.Pose(geo_msgs.Point(-1,1,0), geo_msgs.Quaternion(0,0,0,1))]
    n_locations = len(vertexes)
    i = 0
    location = ""
    last_location = ""
    count = 0

    rospy.loginfo("Starting Simple Square Movement VROOM VROOM")

    while not rospy.is_shutdown():
        i += 1
        if i == n_locations:
            i = 0
        count += 1

        goal = geo_msgs.PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose = vertexes[i]
        # Start the robot toward the next location
        navGoalPub.publish(goal)

        rospy.sleep(150)


    # rostopic pub roy/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'


if __name__ == "__main__":
    try:
        simpleGoalPub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
