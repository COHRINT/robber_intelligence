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
<<<<<<< HEAD
#from resources import floydWarshall 
=======
from resources import floydWarshall
>>>>>>> 1f4b8cd184d479d98ec28b65857a9529c1b3d71c
import matplotlib.pyplot as plt


class CurrentLocation():
	def __init__(self):

		rospy.init_node('costBasdMovement')
		robberName = "roy"

		self.robLoc = geo_msgs.TransformStamped()
		rospy.Subscriber("/" + robberName + "/base_footprint", geo_msgs.TransformStamped, self.getRobberLocation)
		print(self.robLoc) #doesnt work???
		#createGrid(self.robLoc)
		mapPub()

	def getRobberLocation(self, tfMsg):
		poseMsg = geo_msgs.PoseStamped(std_msgs.Header(),
			geo_msgs.Pose(geo_msgs.Point(tfMsg.transform.translation.x, tfMsg.transform.translation.y, tfMsg.transform.translation.z),
			geo_msgs.Quaternion(tfMsg.transform.rotation.x, tfMsg.transform.rotation.y , tfMsg.transform.rotation.z, tfMsg.transform.rotation.w)))
		self.robLoc = poseMsg

def mapPub():
	# 2d Nav Goal
	# ask user for a goal if needed, otherwise automatically set by endpoint
	#rospy.init_node('mapPub')
	rate = rospy.Rate(10)
	curfilePath = os.path.abspath(__file__)
	curDir = os.path.abspath(os.path.join(curfilePath, os.pardir))
	parentDir = os.path.abspath(os.path.join(curDir, os.pardir))

	goal = mov_msgs.MoveBaseGoal()
	goal.target_pose.header.seq = 0
	goal.target_pose.header.stamp = 0
	goal.target_pose.header.frame_id = ""

	mapInfo = parentDir + "/models/map2.yaml"
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
	objNames = {}
	objects = {}
	for item in objDict:
		itemName = item['name']
		if itemName[0:4] != "wall":
			x_loc = item['centroid_x'] + (item['width'] + .3) * math.cos(math.radians(item['orientation']))
			y_loc = item['centroid_y'] + (item['length'] + .3) * math.sin(math.radians(item['orientation']))
			quat = tf.transformations.quaternion_from_euler(0, 0, item['orientation']-180)
			itemLoc = geo_msgs.Pose(geo_msgs.Point(x_loc, y_loc, 0), geo_msgs.Quaternion(quat[0],quat[1],quat[2],quat[3]))
			objLocations[itemName] = itemLoc
			objNames[itemName] = ([item['value']])

	vertexes = objLocations.values()
	vertexKeys = objLocations.keys()
	vertexvalues = objNames.values()

	status = ['PENDING', 'ACTIVE', 'PREEMPTED',
		'SUCCEEDED', 'ABORTED', 'REJECTED',
		'PREEMPTING', 'RECALLING', 'RECALLED',
		'LOST']

	#NAV- STACK
	mover_base = actionlib.SimpleActionClient("roy/move_base", mov_msgs.MoveBaseAction)
	mover_base.wait_for_server(rospy.Duration(5))

	# Keeping track
	n_locations = len(vertexes)

	
	for objKey in objLocations.keys():
		costs = evaluateFloydCost(self.robLoc, floydWarshallCosts, mapGrid, nextPlace, objLocations[objKey])
		newCosts = objNames[objKey] - costs


	#floyd-warshall
	#costs = np.load('floydWarshallCosts.npy')
	#nextPlace = np.load('floydWarshallNextPlace.npy')
	#mapGid = np.load('mapGrid.npy')



	while not rospy.is_shutdown():
		for i in xrange(0, n_locations): #change to iterate through adjusted values
			maximum = max(objNames.iteritems(), key=operator.itemgetter(1))[0]

			if vertexKeys[i] == maximum:
				location = vertexes[i]
				rospy.loginfo("Going to " + maximum)
				break
			else:
				location = vertexes[5]

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
			del objLocations[vertexKeys[i]]
			del objNames[maximum]
		else:
		  rospy.loginfo("Goal failed")
		  rospy.loginfo(status[state])
		  mover_base.cancel_goal()
		  break
		rospy.sleep(1)

def createGrid():

	robberName = "roy"

	# Get list of objects and their locations

	#The current position as retrieved


	#costs = floydWarshall.floyds(grid);
	#floydWarshall.displayMap(costs,pose);

	# Get map information
	gridScale = .05 # size of each grid rectangle compared to map size
	curfilePath = os.path.abspath(__file__)
	curDir = os.path.abspath(os.path.join(curfilePath, os.pardir))
	parentDir = os.path.abspath(os.path.join(curDir, os.pardir))
	mapImgLocation = parentDir + "/models/map2_occupancy.png"
	mapInfoLocation = parentDir + "/models/map2_occupancy.yaml"
	mapInfo = parentDir + '/models/map2.yaml'

	objLocations = getObjects(mapInfo)
	vertexes = objLocations.values()
	vertexKeys = objLocations.keys()
	mapGrid = convertMapToGrid(mapImgLocation, mapInfoLocation, gridScale)

	# Apply floyd warshall algorithm
	#print(len(mapGrid))
	#print(len(mapGrid[0]))
	costs = floydWarshall.floyds(mapGrid)
	np.save('mapGrid', mapGrid)
	np.save('floydWarshallNextPlace', costs)
	plt.imshow(mapGrid, interpolation='nearest')
	plt.show()
	#floydWarshall.displayMap(costs, pose)

def convertMapToGrid(mapImgLocation, mapInfoLocation, gridScale):
	# Get image information
	mapImg = cv2.imread(mapImgLocation)
	pixHeight = mapImg.shape[0]
	pixWidth = mapImg.shape[1]

	# Get resolution (meters per pixel) of map
	with open(mapInfoLocation, 'r') as stream:
		try:
			yamled = yaml.load(stream)
		except yaml.YAMLError as exc:
			print(exc)
	resolution = yamled['resolution']

	# Calculate dimensions of grid
	gridMeterHeight = pixHeight * resolution * gridScale
	gridMeterWidth = pixWidth * resolution * gridScale
	print("Each grid unit is a rectangle with dimensions of " + str(gridMeterHeight) + " x " + str(gridMeterWidth) + " m")

	# Initialize grid
	gridPixelHeight = int(pixHeight*gridScale)
	gridPixelWidth = int(pixWidth*gridScale)
	gridIteratorY = range(0, pixHeight, gridPixelHeight)
	gridIteratorX = range(0, pixWidth, gridPixelWidth)
	grid = np.zeros((len(gridIteratorY), len(gridIteratorX)))
	# grid = np.zeros((int(pixHeight/(pixHeight*gridScale)), int(pixWidth/(pixWidth*gridScale))))

	# Populate grid
	iGrid = 0
	for i in gridIteratorY:
		jGrid = 0
		for j in gridIteratorX:
			rect = mapImg[i:i+gridPixelHeight, j:j+gridPixelWidth]
			if rect.mean() < 255:
				grid[iGrid][jGrid] = 1
			jGrid+=1
		iGrid+=1
	return grid


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
			itemLoc = geo_msgs.Pose(geo_msgs.Point(x_loc, y_loc, 0), geo_msgs.Quaternion(quat[0],quat[1],quat[2],quat[3]))
			objLocations[itemName] = itemLoc
	return objLocations

def path(ux, uy, vx, vy, nextPlace):
	if nextPlace[ux, uy, vx, vy] == None:
		return []
	path = [(ux, uy)]
	while (ux != vx) and (uy != vy):
		ux, uy = nextPlace[ux, uy, vx, vy]
		path.append(u)
	return path

def convertPositionToGrid(x,y,grid): #objects and robber
	SizeY, SizeX = 0.36, 0.68
	originY = -3.6
	originX = -9.6
	y += -1*originY
	x += -1*originX
	gridX = int(x/SizeX)
	gridY = int(y/SizeY)
	return gridX, gridY

def evaluateFloydCost(robLoc, floydWarshallCosts, mapGrid, nextPlace, objects):
	robGridLocY, robGridLocX = convertPositionToGrid(robLoc.pose.position.x, robLoc.pose.position.y, mapGrid)
	# print(str(copGridLocX) + " " + str(copGridLocY))

	objGridLocY, objGridLocX = convertPositionToGrid(objects.position.x, objects.position.y, mapGrid)

	cost = floydWarshallCosts[objGridLocY][objGridLocX][poseGridLocY][poseGridLocX]

	return cost

def main():
	CurrentLocation()
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
