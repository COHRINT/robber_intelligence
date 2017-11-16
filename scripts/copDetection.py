#!/usr/bin/env python

# Builds the costmap by evaluating where the cop is and its distance to everywhere else in the map

import yaml
import random
import math
import tf
import rospy
# import geometry_msgs
# import nav_msgs
# import move_base_msgs
# import actionlib
import actionlib_msgs.msg as act_msgs # import *
import geometry_msgs.msg as geo_msgs # geo_msgs.Pose, geo_msgs.PoseWithCovarianceStamped, geo_msgs.Point, geo_msgs.Quaternion, geo_msgs.Twist, geo_msgs.Vector3
import move_base_msgs.msg as mov_msgs # import mov_msgs.MoveBaseAction, mov_msgs.MoveBaseGoal
import nav_msgs.msg as nav_msgs
import std_msgs.msg as std_msgs

from PIL import Image

def copDetection():
    tol = .1
    robberName = "roy"

    robLoc = geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(1,1,0), geo_msgs.Quaternion(0,0,0,1)))
    destination = geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(0,1,0), geo_msgs.Quaternion(0,0,0,1)))

    print(robLoc)
    print(destination)

    # Get plan from make_plan service
    rospy.wait_for_service("/" + robberName + "move_base/make_plan")
    try:
        planner = rospy.ServiceProxy("/" + robberName + "move_base/make_plan", nav_msgs.GetPlan)
        plan = planner(robLoc, destination, tol)
        poses = plan.plan.poses
        print(poses)
    except rospy.ServiceException, e:
        print "GetPlan service call failed: %s"%e
        return 0


if __name__ == '__main__':
    try:
        copDetection()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
