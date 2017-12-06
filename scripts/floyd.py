#!/usr/bin/env python

# Builds the costmap by evaluating where the cop is and its distance to everywhere else in the map

import yaml
import random
import math
import tf
import numpy as np
import matplotlib.pyplot as plt
import rospy
import actionlib_msgs.msg as act_msgs # import *
import geometry_msgs.msg as geo_msgs # geo_msgs.Pose, geo_msgs.PoseWithCovarianceStamped, geo_msgs.Point, geo_msgs.Quaternion, geo_msgs.Twist, geo_msgs.Vector3
import move_base_msgs.msg as mov_msgs # import mov_msgs.MoveBaseAction, mov_msgs.MoveBaseGoal
import std_msgs.msg as std_msgs
import nav_msgs.srv as nav_srv
import nav_msgs.msg as nav_msgs
from resources import floydWarshall
from PIL import Image
import os.path
import cv2


def copDetection():

    copName = "deckard"
    robberName = "roy"

    # Get list of objects and their locations
    mapInfo = 'map2.yaml'
    objLocations = getObjects(mapInfo)
    vertexes = objLocations.values()
    vertexKeys = objLocations.keys()

    #A 5x5 grid with a wall near the top
    grid = [[0,0,0,0,0],[0,1,1,1,0],[0,0,0,1,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]

    #The cops position
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
    # floydWarshall.displayMap(costs, pose)





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
        copDetection()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
