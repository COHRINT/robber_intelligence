#!/usr/bin/env python

# Builds the costmap by evaluating where the cop is and its distance to everywhere else in the map

import yaml
import math
import tf
import numpy as np
import matplotlib.pyplot as plt
import rospy
import os.path
import scipy.stats

import actionlib
import actionlib_msgs.msg as act_msgs # import *
import geometry_msgs.msg as geo_msgs # geo_msgs.Pose, geo_msgs.PoseWithCovarianceStamped, geo_msgs.Point, geo_msgs.Quaternion, geo_msgs.Twist, geo_msgs.Vector3
import move_base_msgs.msg as mov_msgs # import mov_msgs.MoveBaseAction, mov_msgs.MoveBaseGoal
import std_msgs.msg as std_msgs
import nav_msgs.srv as nav_srv
import nav_msgs.msg as nav_msgs

class robberEvasion():

	def __init__(self, copName, robberName):
		# Setup node
		rospy.init_node('robberEvasion')
		#rospy.on_shutdown(self.shutDown) # Not Working

		# robberGoalPub = rospy.Publisher('robberGoalPub', robber_intelligence/RobberEvasion, queue_size=10)


		# Setup robber service
		robberSrv = rospy.Service('robberEvasionGoal', robberEvasionGoal, self.handleRobberSrv)
		self.curRobberGoal = geo_msgs.PoseStamped()

		# Retrieve robot locations
		rospy.Subscriber("/" + copName + "/base_footprint", geo_msgs.TransformStamped, self.getCopLocation)
		rospy.Subscriber("/" + robberName + "/base_footprint", geo_msgs.TransformStamped, self.getRobberLocation)
		self.copLoc = geo_msgs.PoseStamped()
		self.pastCopLoc = geo_msgs.PoseStamped()
		self.robLoc = geo_msgs.PoseStamped()

		# Setup nav stack
		self.mover_base = actionlib.SimpleActionClient(robberName + "/move_base", mov_msgs.MoveBaseAction)
		self.mover_base.wait_for_server(rospy.Duration(5))
		status = ['PENDING', 'ACTIVE', 'PREEMPTED',
		'SUCCEEDED', 'ABORTED', 'REJECTED',
		'PREEMPTING', 'RECALLING', 'RECALLED',
		'LOST']

		# Get list of objects, locations, and values
		curfilePath = os.path.abspath(__file__)
		curDir = os.path.abspath(os.path.join(curfilePath, os.pardir))
		parentDir = os.path.abspath(os.path.join(curDir, os.pardir))
		mapInfo = parentDir + '/models/map2.yaml'
		self.objLocations, self.objNames = getObjects(mapInfo)
		vertexes = self.objLocations.values()
		vertexKeys = self.objLocations.keys()
		vertexvalues = self.objNames.values()

		# Load Floyd Warshall info
		self.mapGrid = np.load(parentDir + '/resources/mapGrid.npy')
		self.floydWarshallCosts = np.load(parentDir + '/resources/floydWarshallCosts20.npy')
		self.floydWarshallNextPlace = np.load(parentDir + '/resources/floydWarshallNextPlace.npy')

		# Evasion Parameters
		reevaluationTime = 3 # Time to wait before reevaluating the path robber is following
		dangerWeight = 0.75 # Amount of danger before robber should choose a new path

		# Map Parameters
		self.originY, self.originX = -3.6, -9.6 # ????
		# mapSizeY, mapSizeX = 0.18, 0.34
		self.mapSizeY, self.mapSizeX = 0.36, 0.68 # ?? is this map size in meters???

		# Distributions of cost/reward measures
        copSafetyMean = 206.06
        copSafetyStdDev = 148.51
        copSafetyDistribution = scipy.stats.norm(copSafetyMean, copSafetyStdDev) #query with copSafetyDistribution.cdf(value)

		#determine adjusted costs [OLD]
		for objKey in self.objLocations.keys():
			costs = self.evaluateFloydCostBased(self.objLocations[objKey])
			rospy.loginfo(self.objNames[objKey])

		#[NEW]
		costMax, meanCost, stdCost = self.findMaxCostBased()
		costDistribution = scipy.stats.norm(meanCost,stdCost)
		#ospy.loginfo(self.findMaxCostBased())
		rospy.loginfo(costDistribution.cdf(costMax))

		# Begin Evasion
		while not rospy.is_shutdown():
			# Choose destination of least cost
			curCost, curDestination = self.floydChooseDestination()
			rospy.loginfo("Stealing goods at " + curDestination + " with danger level of " + str(curCost))

			# Travel to destination
			goal = mov_msgs.MoveBaseGoal()
			goal.target_pose.pose = self.objLocations[curDestination].pose
			goal.target_pose.header.frame_id = 'map'
			goal.target_pose.header.stamp = rospy.Time.now()
			rospy.loginfo(goal)
            # TODO: send goal here to robber_evasion_planner using pub/sub or srv?
			self.curRobberGoal = self.objLocations[curDestination].pose

			self.mover_base.send_goal(goal)



			# Would this make it so you wait for 2 seconds every time?
			# mover_base.wait_for_result(rospy.Duration(2))


			# While robber is travelling to destination, evaluate the path it is following every few seconds
			state = self.mover_base.get_state()
			pathFailure = False
			while (pathFailure==False): #(status[state]=='PENDING' or status[state]=='ACTIVE') and
				# Evaluate cost of path
				newCost = self.evaluateFloydCost(self.objLocations[curDestination])
				print ("New Cost: " + str(newCost))

				# Check if path is too dangerous
				if newCost < curCost*dangerWeight:
					pathFailure = True

				# Display Costmap
				copGridLocY, copGridLocX = self.convertPoseToGridLocation(self.copLoc.pose.position.y , self.copLoc.pose.position.x)
				#plt.imshow(self.floydWarshallCosts[copGridLocY][copGridLocX]);
				#plt.ion()
				#plt.show();
				#plt.pause(.0001)

				# Pause for few seconds until reevalutation of path
				rospy.sleep(reevaluationTime)
				state = self.mover_base.get_state()

			# Check what robber has accomplished
			if pathFailure == True: # Robber path is too dangerous, choose a new path
				rospy.loginfo("Path is too dangerous, finding a new object to steal.")
				self.mover_base.cancel_goal()
			elif status[state] != 'SUCCEEDED': # Failure in getting to object
				rospy.loginfo("Robber failed to reach object with error code " + str(state) + ": " + status[state] + ". Finding something else to steal.")
			else: # SUCCESSFUL ROBBERY
				rospy.loginfo("MWUAHAHAHAHAHA You've successfully stolen valuable goods from the " + curDestination)

	# Sends goal of robber to robber_evasion_planner
	def handleRobberSrv(self, req):
		if req.isRunning:
	    	return robberEvasionGoalResponse(self.curRobberGoal)


	# Goes through entire list of objects and returns object with path that is least likely to be detected and its cost
	def floydChooseDestination(self):
		maxDist = 0
		maxDistLocation = ""
		for objKey in self.objLocations.keys():
			objCost = self.evaluateFloydCost(self.objLocations[objKey])
			print(objKey + ": " + str(objCost))
			if objCost > maxDist:
				maxDist = objCost
				maxDistLocation = objKey
		return maxDist, maxDistLocation


	# ALSO need to comment this....
	def evaluateFloydCost(self, objPose):
		copGridLocY, copGridLocX = self.convertPoseToGridLocation(self.copLoc.pose.position.y , self.copLoc.pose.position.x)
		robGridLocY, robGridLocX = self.convertPoseToGridLocation(self.robLoc.pose.position.y, self.robLoc.pose.position.x)
		poseGridLocY, poseGridLocX = self.convertPoseToGridLocation(objPose.pose.position.y, objPose.pose.position.x)
		path = self.makePath(robGridLocY, robGridLocX, poseGridLocY, poseGridLocX)
		cost = 0
		for point in path:
			poseGridLocY, poseGridLocX = point
			pointCost = self.floydWarshallCosts[copGridLocY][copGridLocX][poseGridLocY][poseGridLocX]
			while pointCost == np.Inf:
				poseGridLocY+=1
				if poseGridLocY>39:
					poseGridLocY = 0
				pointCost = self.floydWarshallCosts[copGridLocY][copGridLocX][poseGridLocY][poseGridLocX]
			cost += pointCost
		return cost

	def evaluateFloydCostBased(self, objects):
		robGridLocY, robGridLocX = self.convertPoseToGridLocation(self.robLoc.pose.position.y, self.robLoc.pose.position.x)
		# print(str(copGridLocX) + " " + str(copGridLocY))

		objGridLocY, objGridLocX = self.convertPoseToGridLocation(objects.pose.position.y, objects.pose.position.x)

		cost = self.floydWarshallCosts[robGridLocY][robGridLocX][objGridLocY][objGridLocX]

		return cost

	def findMaxCostBased(self): #Max is approx. 80 from file cabinet, (mean=2.7, std_dev=31.4)
		floydSize = self.floydWarshallCosts.shape
		maxCost = 0
		costArray = []
		print("Finding mean, std deviation of costs")
		# Need to run through each location of robber
		# Run through every cell in floydGrid
		for i in range(floydSize[0]): # go through robber locations
			for j in range(floydSize[1]):
				for objKey in self.objLocations.keys(): # go through objects
					poseGridLocY, poseGridLocX = self.convertPoseToGridLocation(self.objLocations[objKey].pose.position.y, self.objLocations[objKey].pose.position.x)
					cost = self.floydWarshallCosts[i][j][poseGridLocY][poseGridLocX]
					if cost != np.inf: #exclude wall locations
						cost = (-1*cost) + self.objNames[objKey]
						#rospy.loginfo(cost)
						costArray.append(cost)
					 #if we're going to normalize anyway, consider the value dimensionless in which case conversion doesn't matter
		return max(costArray), np.mean(costArray), np.std(costArray)

	# I need to comment this, investigate floyd algorithm file
	def convertPoseToGridLocation(self, y, x):
		y += -1*self.originY
		x += -1*self.originX
		# mapGridDimY, mapGridDimX = self.mapGrid.shape
		gridLocY = int(y / self.mapSizeY)
		gridLocX = int(x / self.mapSizeX)
		return gridLocY, gridLocX

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


	def makePath(self, ux, uy, vx, vy):
		if self.floydWarshallNextPlace[ux, uy, vx, vy] == None:
			return []
		path = [(ux, uy)]
		while (ux != vx) or (uy != vy):
			ux, uy = self.floydWarshallNextPlace[ux, uy, vx, vy]
			path.append((ux, uy))
		return path

	def countPath(self, ux, uy, vx, vy):
		x = 0
		y = 0
		startX = ux
		startY = uy
		if self.floydWarshallNextPlace[ux, uy, vx, vy] == None:
			return []
		path = [(ux, uy)]
		while (ux != vx) or (uy != vy):
			ux, uy = self.floydWarshallNextPlace[ux, uy, vx, vy]
			path.append((ux, uy))
			x = x + ux - startX
			y = y + uy - startY
		return x,y

	def shutDown(self):
		rospy.loginfo("Stopping the robot...")
		self.mover_base.cancel_goal()
		# rospy.sleep(2)
		# self.cmd_vel_pub.publish(Twist())
		rospy.is_shutdown()



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
	objNames = {}
	for item in objDict:
		itemName = item['name']
		if itemName[0:4] != "wall":
			x_loc = item['centroid_x'] + (item['width']/2 + .6) * math.cos(math.radians(item['orientation']))
			y_loc = item['centroid_y'] + (item['length']/2 + .6) * math.sin(math.radians(item['orientation']))
			quat = tf.transformations.quaternion_from_euler(0, 0, item['orientation']-180)
			itemLoc = geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(x_loc, y_loc, 0), geo_msgs.Quaternion(quat[0],quat[1],quat[2],quat[3])))
			objLocations[itemName] = itemLoc
			objNames[itemName] = ([item['value']])
	return objLocations, objNames


def main():
	robberEvasion(copName="zhora", robberName="deckard")
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
