#!/usr/bin/env python

import yaml
import random
import math
import tf
import numpy as np
import scipy.stats
import rospy
# import geometry_msgs
# import nav_msgs
# import move_base_msgs
# import actionlib
import actionlib_msgs.msg as act_msgs # import *
import geometry_msgs.msg as geo_msgs # geo_msgs.Pose, geo_msgs.PoseWithCovarianceStamped, geo_msgs.Point, geo_msgs.Quaternion, geo_msgs.Twist, geo_msgs.Vector3
import move_base_msgs.msg as mov_msgs # import mov_msgs.MoveBaseAction, mov_msgs.MoveBaseGoal
import std_msgs.msg as std_msgs
import nav_msgs.srv as nav_srv
import nav_msgs.msg as nav_msgs


class copDetection():

    def __init__(self):
        copName = "deckard"
        robberName = "roy"

        rospy.Subscriber("/" + copName + "/", geo_msgs.Pose, self.getCopLocation)
        rospy.Subscriber("/" + robberName + "/", geo_msgs.Pose, self.getRobberLocation)

        print(self.copLoc)
        print(self.robLoc)


    def getCopLocation(self, poseMsg):
        self.copLoc = poseMsg

    def getRobberLocation(self, poseMsg):
        self.robLoc = poseMsg


if __name__ == '__main__':
    try:
        copDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
