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

class CurrentLoctation():

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

def mapPub():
    # 2d Nav Goal
    # ask user for a goal if needed, otherwise automatically set by endpoint
    rospy.init_node('mapPub')
    rate = rospy.Rate(10)

    goal = mov_msgs.MoveBaseGoal()
    goal.target_pose.header.seq = 0
    goal.target_pose.header.stamp = 0
    goal.target_pose.header.frame_id = ""

    mapInfo = 'map2.yaml'
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

    mover_base = actionlib.SimpleActionClient("deckard/move_base", mov_msgs.MoveBaseAction)
    mover_base.wait_for_server(rospy.Duration(5))

    # Keeping track
    n_locations = len(vertexes)
    

    rospy.loginfo("vroom")
    c = CurrentLoctation()
    #print(c.poseMsg)
    #floyd-warshall


    # Go through the series of locations indefinitely
    while not rospy.is_shutdown():
    	for i in xrange(0, n_locations):
    		maximum = max(objNames.iteritems(), key=operator.itemgetter(1))[0]

        	if vertexKeys[i] == maximum:
        		location = vertexes[i]
        		rospy.loginfo("Going to " + maximum)
        		break
        	else:
        		location = vertexes[5]
        		
        # Set up goa5
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
    mapInfo = 'map2.yaml'
    objLocations = getObjects(mapInfo)
    vertexes = objLocations.values()
    vertexKeys = objLocations.keys()

    #A 5x5 grid with a wall near the top
    grid = [[0,0,0,0,0],[0,1,1,1,0],[0,0,0,1,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]

    #The current position as retrieved
    pose = [0,3]; 

    # costs = floydWarshall.floyds(grid);
    #floydWarshall.displayMap(costs,pose);

    # Get map information
    gridScale = .01 # size of each grid rectangle compared to map size
    curfilePath = os.path.abspath(__file__)
    curDir = os.path.abspath(os.path.join(curfilePath, os.pardir))
    parentDir = os.path.abspath(os.path.join(curDir, os.pardir))
    mapImgLocation = parentDir + "/models/map2_occupancy.png"
    mapInfoLocation = parentDir + "/models/map2_occupancy.yaml"
    mapGrid = convertMapToGrid(mapImgLocation, mapInfoLocation, gridScale)

    # Apply floyd warshall algorithm
    print(len(mapGrid))
    print(len(mapGrid[0]))
    costs = floydWarshall.floyds(mapGrid)
    np.save('mapGrid', mapGrid)
    np.save('floydWarshallCosts', costs)
    plt.imshow(mapGrid, interpolation='nearest')
    plt.show()
    floydWarshall.displayMap(costs, pose)





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
    print("Each grid unit is a rectangle with dimensions of " + str(gridMeterHeight) + "x" + str(gridMeterWidth) + " m")

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
            # print(rect)
            # print(rect.mean())
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




if __name__ == '__main__':
    try:
        mapPub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
