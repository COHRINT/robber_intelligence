#!/usr/bin/env python

# Builds the costmap by evaluating where the cop is and its distance to everywhere else in the map

import yaml
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
import os.path
import cv2


def floyd():
    # Get map information
    gridScale = .025 # size of each grid rectangle compared to map size
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
            if rect.mean() < 200:
                grid[iGrid][jGrid] = 1
            jGrid+=1
        iGrid+=1

    return grid

if __name__ == '__main__':
    try:
        floyd()
    except rospy.ROSInterruptException:
        pass
