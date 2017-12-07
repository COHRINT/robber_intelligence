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


class robotLocation():

    def __init__(self):
        rospy.init_node('robotLoc')
        rate = rospy.Rate(10)

        copName = "deckard"
        robberName = "roy"
        # self.copLoc = geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(1,1,0), geo_msgs.Quaternion(0,0,1,0)))
        self.robLoc = geo_msgs.TransformStamped()

        rospy.Subscriber("/" + copName + "/base_footprint", geo_msgs.TransformStamped, self.getCopLocation)
        rospy.Subscriber("/" + robberName + "/base_footprint", geo_msgs.TransformStamped, self.getRobberLocation)

        while not rospy.is_shutdown():
            # print(self.copLoc)
            print(self.robLoc)


    def getCopLocation(self, tfMsg):
        poseMsg = geo_msgs.PoseStamped(std_msgs.Header(), 
            geo_msgs.Pose(geo_msgs.Point(tfMsg.transform.translation.x, tfMsg.transform.translation.y, tfMsg.transform.translation.z), 
            geo_msgs.Quaternion(tfMsg.transform.rotation.x, tfMsg.transform.rotation.y , tfMsg.transform.rotation.z, tfMsg.transform.rotation.w)))
        self.copLoc = poseMsg


    def getRobberLocation(self, tfMsg):
        poseMsg = geo_msgs.PoseStamped(std_msgs.Header(), 
            geo_msgs.Pose(geo_msgs.Point(tfMsg.transform.translation.x, tfMsg.transform.translation.y, tfMsg.transform.translation.z), 
            geo_msgs.Quaternion(tfMsg.transform.rotation.x, tfMsg.transform.rotation.y , tfMsg.transform.rotation.z, tfMsg.transform.rotation.w)))
        self.robLoc = poseMsg


if __name__ == '__main__':
    try:
        robotLocation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
