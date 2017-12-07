#!/usr/bin/env python

# Builds the costmap by evaluating where the cop is and its distance to everywhere else in the map

import yaml
import math
import tf
import numpy as np
import matplotlib.pyplot as plt
import rospy

import actionlib
import actionlib_msgs.msg as act_msgs # import *
import geometry_msgs.msg as geo_msgs # geo_msgs.Pose, geo_msgs.PoseWithCovarianceStamped, geo_msgs.Point, geo_msgs.Quaternion, geo_msgs.Twist, geo_msgs.Vector3
import move_base_msgs.msg as mov_msgs # import mov_msgs.MoveBaseAction, mov_msgs.MoveBaseGoal
import std_msgs.msg as std_msgs
import nav_msgs.srv as nav_srv
import nav_msgs.msg as nav_msgs

class copDetection():

	def __init__(self, copName, robberName):
		# Setup node
		rospy.init_node('robotLoc')
		# Retrieve robot locations
		rospy.Subscriber("/" + copName + "/base_footprint", geo_msgs.TransformStamped, self.getCopLocation)
		rospy.Subscriber("/" + robberName + "/base_footprint", geo_msgs.TransformStamped, self.getRobberLocation)
		self.copLoc = geo_msgs.PoseStamped()
		self.pastCopLoc = geo_msgs.PoseStamped()
		self.robLoc = geo_msgs.PoseStamped()

		# Setup nav stack
		# goal = mov_msgs.MoveBaseGoal()
		# goal.target_pose.header.seq = 0
		# goal.target_pose.header.stamp = 0
		# goal.target_pose.header.frame_id = ""
		status = ['PENDING', 'ACTIVE', 'PREEMPTED',
		'SUCCEEDED', 'ABORTED', 'REJECTED',
		'PREEMPTING', 'RECALLING', 'RECALLED',
		'LOST']
		mover_base = actionlib.SimpleActionClient(robberName + "/move_base", mov_msgs.MoveBaseAction)
		mover_base.wait_for_server(rospy.Duration(5))


		# Get list of objects and their locations
		mapInfo = 'map2.yaml'
		objLocations = getObjects(mapInfo)
		vertexes = objLocations.values()
		vertexKeys = objLocations.keys()


		# Load Floyd Warshall info
		mapGrid = np.load('mapGrid.npy')
		floydWarshallCosts = np.load('floydWarshallCosts.npy')


		while not rospy.is_shutdown():
			# Choose destination
			curCost, curDestination = floydChooseDestination(objLocations, self.copLoc, self.robLoc, floydWarshallCosts, mapGrid)
			rospy.loginfo("Stealing goods at " + curDestination + " with danger level of " + str(curCost))

			# Travel to destination
			goal = mov_msgs.MoveBaseGoal()
			goal.target_pose.pose = objLocations[curDestination]
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.header.stamp = rospy.Time.now()
			rospy.loginfo(goal)
			mover_base.send_goal(goal)


			# While robber is travelling to destination, evaluate the path it is following every 1 second
			state = mover_base.get_state()
			pathFailure = False
			while (state==1 or state==0) and (pathFailure==False): # ACTIVE
				newCost = evaluateFloydCost(self.copLoc, objLocations[curDestination], floydWarshallCosts, mapGrid)
				if newCost > curCost*2:
					pathFailure = True
				rospy.sleep(1)
				state = mover_base.get_state()
			if pathFailure==True:
				rospy.loginfo("Path is too dangerous, finding a new object to steal.")
				mover_base.cancel_goal()
			elif state!=3:
				rospy.loginfo("Robber failed to reach object with error code " + str(state) + ": " + status[state] + ". Finding something else to steal.")
			else:
				rospy.loginfo("MWUAHAHAHAHAHA You've successfully stolen valuable goods from the " + objKey)



	def getCopLocation(self, tfMsg):
		self.pastCopLoc = self.copLoc
		poseMsg = geo_msgs.PoseStamped(std_msgs.Header(), 
			geo_msgs.Pose(geo_msgs.Point(tfMsg.transform.translation.x, tfMsg.transform.translation.y, tfMsg.transform.translation.z), 
			geo_msgs.Quaternion(tfMsg.transform.rotation.x, tfMsg.transform.rotation.y , tfMsg.transform.rotation.z, tfMsg.transform.rotation.w)))
		self.copLoc = poseMsg


	def getRobberLocation(self, tfMsg):
		poseMsg = geo_msgs.PoseStamped(std_msgs.Header(), 
			geo_msgs.Pose(geo_msgs.Point(tfMsg.transform.translation.x, tfMsg.transform.translation.y, tfMsg.transform.translation.z), 
			geo_msgs.Quaternion(tfMsg.transform.rotation.x, tfMsg.transform.rotation.y , tfMsg.transform.rotation.z, tfMsg.transform.rotation.w)))
		self.robLoc = poseMsg


def floydChooseDestination(objLocations, copLoc, robLoc, floydWarshallCosts, mapGrid):
	maxDist = 0
	maxDistLocation = ""
	for objKey in objLocations.keys():
		objCost = evaluateFloydCost(copLoc, objLocations[objKey], floydWarshallCosts, mapGrid)
		if objCost > maxDist:
			maxDist = objCost
			maxDistLocation = objKey
	return maxDist, maxDistLocation

def evaluateFloydCost(copLoc, pose, floydWarshallCosts, mapGrid):
    copGridLocY, copGridLocX = convertPoseToGridLocation(copLoc.pose.position.y , copLoc.pose.position.x, mapGrid)
    poseGridLocY, poseGridLocX = convertPoseToGridLocation(pose.pose.position.y, pose.pose.position.x, mapGrid)
    cost = floydWarshallCosts[copGridLocY][copGridLocX][poseGridLocY][poseGridLocX]
    while cost == np.Inf:
    	poseGridLocY+=1
    	if poseGridLocY>39:
    		poseGridLocY = 0
    	cost = floydWarshallCosts[copGridLocY][copGridLocX][poseGridLocY][poseGridLocX]
    return cost

def convertPoseToGridLocation(y, x, grid):
    originY = -3.6
    originX = -9.6
    y += -1*originY
    x += -1*originX
    mapSizeY, mapSizeX = 0.18, 0.34
    mapGridDimY, mapGridDimX = grid.shape
    gridLocY = int(y / mapSizeY)
    gridLocX = int(x / mapSizeX)
    return gridLocY, gridLocX

def getObjects(mapInfo):
    with open(mapInfo, 'r') as stream:
        try:
            yamled = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # deletes info
    del yamled['info']

    # Populates dictionary with location names and their poses
    objDict = yamled.values()
    objLocations = {}
    for item in objDict:
        itemName = item['name']
        if itemName[0:4] != "wall":
            x_loc = item['centroid_x'] + (item['width']/2 + .6) * math.cos(math.radians(item['orientation']))
            y_loc = item['centroid_y'] + (item['length']/2 + .6) * math.sin(math.radians(item['orientation']))
            quat = tf.transformations.quaternion_from_euler(0, 0, item['orientation']-180)
            itemLoc = geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(x_loc, y_loc, 0), geo_msgs.Quaternion(quat[0],quat[1],quat[2],quat[3])))
            objLocations[itemName] = itemLoc
    return objLocations


def main():
	copDetection(copName="deckard", robberName="roy")
    # rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
