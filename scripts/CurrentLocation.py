#!/usr/bin/env python

import yaml
import random
import math
import tf
import rospy
import geometry_msgs
import operator
import nav_msgs
import move_base_msgs
import actionlib
import actionlib_msgs.msg as act_msgs # import *
import geometry_msgs.msg as geo_msgs # geo_msgs.Pose, geo_msgs.PoseWithCovarianceStamped, geo_msgs.Point, geo_msgs.Quaternion, geo_msgs.Twist, geo_msgs.Vector3
import move_base_msgs.msg as mov_msgs # import mov_msgs.MoveBaseAction, mov_msgs.MoveBaseGoal
import os.path
import cv2
import numpy as np
import std_msgs.msg as std_msgs

class CurrentLocation():
    def __init__(self):
        robberName = "deckard"

        self.robLoc = geo_msgs.TransformStamped()
        rospy.Subscriber("/" + robberName + "/base_footprint", geo_msgs.TransformStamped, self.getRobberLocation)
        print(self.robLoc)

    def getRobberLocation(self, tfMsg):
        poseMsg = geo_msgs.PoseStamped(std_msgs.Header(), 
            geo_msgs.Pose(geo_msgs.Point(tfMsg.transform.translation.x, tfMsg.transform.translation.y, tfMsg.transform.translation.z), 
            geo_msgs.Quaternion(tfMsg.transform.rotation.x, tfMsg.transform.rotation.y , tfMsg.transform.rotation.z, tfMsg.transform.rotation.w)))
	self.robLoc = poseMsg

    def getLoc():
        return(self.robLoc)