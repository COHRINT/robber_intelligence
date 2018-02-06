#!/usr/bin/env python

"""
*************************************************
File: floyd.py
Author: Sousheel Vunnam, Luke Burks
Date: November 2017

Demonstrating a variation on the floyd warshall
algorithm and applying it to the evasion problem

*************************************************
"""

# Builds the costmap by evaluating where the cop is and its distance to everywhere else in the map

import math
import numpy as np
import matplotlib.pyplot as plt
import os.path
import cv2
import yaml
import tf
import geometry_msgs.msg as geo_msgs
import std_msgs.msg as std_msgs


def main():
    # Get map directory
    curfilePath = os.path.abspath(__file__)
    curDir = os.path.abspath(os.path.join(curfilePath, os.pardir))
    parentDir = os.path.abspath(os.path.join(curDir, os.pardir))

    # Map parameters
    mapImgLocation = parentDir + "/models/map2_occupancy.png"
    mapImgInfoLocation = parentDir + "/models/map2_occupancy.yaml"
    mapInfoLocation = parentDir + "/models/map2.yaml"
    gridScale = 0.05 # size of each grid rectangle compared to map size (makes a 20x20 grid)

    costs, nextPlace, gridY, gridX = floydWarshallAlgorithm(mapImgLocation, mapImgInfoLocation, gridScale)
    createYaml(mapInfoLocation, costs, nextPlace, gridY, gridX)


def floydWarshallAlgorithm(mapImgLocation, mapInfoLocation, gridScale):
    # Convert map to floyd grid
    mapGrid, gridY, gridX = convertMapToGrid(mapImgLocation, mapInfoLocation, gridScale)

    # Apply floyd warshall algorithm
    print("Created a " + str(len(mapGrid)) + " by " + str(len(mapGrid[0])) + " map grid.")
    costs, nextPlace = floyds(mapGrid)
    np.save('mapGrid', mapGrid)
    np.save('floydWarshallCosts', costs)
    np.save('floydWarshallNextPlace', nextPlace)
    # Display map
    # plt.imshow(mapGrid, interpolation='nearest')
    # plt.show()
    # floydWarshall.displayMap(costs, pose)

    return costs, nextPlace, gridY, gridX



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

    return grid, gridMeterHeight, gridMeterWidth


def floyds(grid):
	'''
	Runs a version of the floyd warshall algorithm

	Inputs:
	#Grid: Occupancy grid. List of lists, or 2D numpy array, of 0's and 1's
	#pose: cops position, length 2 list [x,y]

	Outputs:
	#dist: Grid of distance, numpy array

	'''

	#find dimensions of grid
	sizeX = len(grid);
	sizeY = len(grid[0]);

	#initialize distance grid to infinity
	dist = np.ones(shape = (sizeX,sizeY,sizeX,sizeY))*np.Inf;
	nextPlace = np.empty(shape = (sizeX,sizeY,sizeX,sizeY), dtype=object);

	#enforce that cells are zero distance from themselves
	for i in range(0,sizeX):
		for j in range(0,sizeY):
			dist[i,j,i,j] = 0;

	#set the distance of each cell to it's neighbors as 1
	#only cardinal directions, no diagonals for simplicity
	#makes sure occupied cells can't be accessed
	for i in range(0,sizeX):
		for j in range(0,sizeY):
			if(i>0 and grid[i-1][j] == 0):
				dist[i,j,i-1,j] = 1;
				nextPlace[i,j,i-1,j] = (i-1, j)
			if(i<sizeX-1 and grid[i+1][j] == 0):
				dist[i,j,i+1,j] = 1;
				nextPlace[i,j,i+1,j] = (i+1, j)
			if(j>0 and grid[i][j-1] == 0):
				dist[i,j,i,j-1] = 1;
				nextPlace[i,j,i,j-1] = (i, j-1)
			if(j<sizeY-1 and grid[i][j+1] == 0):
				dist[i,j,i,j+1] = 1;
				nextPlace[i,j,i,j+1] = (i, j+1)

	#Main loop, pretty ugly...
	#but a simple if statement at the core
	for kx in range(0,sizeX):
		for ky in range(0,sizeY):
			for ix in range(0,sizeX):
				for iy in range(0,sizeY):
					for jx in range(0,sizeX):
						for jy in range(0,sizeY):
							if(dist[ix,iy,jx,jy] > dist[ix,iy,kx,ky] + dist[kx,ky,jx,jy]):
								dist[ix,iy,jx,jy] = dist[ix,iy,kx,ky] + dist[kx,ky,jx,jy];
								nextPlace[ix, iy, jx, jy] = nextPlace[ix, iy, kx, ky]
		print("Iteration " + str(kx) + " of " + str(sizeX))

	return dist, nextPlace

def path(ux, uy, vx, vy, nextPlace):
    if nextPlace[ux, uy, vx, vy] == None:
        return []
    path = [(ux, uy)]
    while (ux != vx) and (uy != vy):
        ux, uy = nextPlace[ux, uy, vx, vy]
        path.append(u)
    return path

def makePath(floydWarshallNextPlace, ux, uy, vx, vy):
    if floydWarshallNextPlace[ux, uy, vx, vy] == None:
        return []
    path = [(ux, uy)]
    while (ux != vx) or (uy != vy):
        ux, uy = floydWarshallNextPlace[ux, uy, vx, vy]
        path.append((ux, uy))
    return path


def createYaml(mapInfoLocation, floydWarshallCosts, floydWarshallNextPlace, gridY, gridX):
    # Get list of objects, locations, and values
    objLocations, objNames, originY, originX = getObjects(mapInfoLocation)

    meanValue, stdValue = findMaxValueBased(objLocations, objNames, floydWarshallCosts, originY, originX, gridY, gridX)
    meanCopCost, stdCopCost = findMaxCopCost(objLocations, objNames, floydWarshallCosts, floydWarshallNextPlace, originY, originX, gridY, gridX)

    floydYaml = {
        'originY': originY,
        'originX': originX,
        'mapSizeY': gridY,
        'mapSizeX': gridX,
        'meanObjectValue': float(meanValue),
        'stdObjectValue': float(stdValue),
        'meanCopCost': float(meanCopCost),
        'stdCopCost': float(stdCopCost)
    }
    with open('floydInfo.yaml', 'w') as yaml_file:
        yaml.dump(floydYaml, yaml_file, default_flow_style=False)

def displayMap(costs,pose=[0,3]):
	#lets see what the cost looks like for a position
	plt.imshow(costs[0,3]);
	plt.show();


def findMaxCopCost(objLocations, objNames, floydWarshallCosts, floydWarshallNextPlace, originY, originX, mapSizeY, mapSizeX): #Max is 1083, (mean=206.05962, std_dev=148.51310204559596)
    floydSize = floydWarshallCosts.shape
    maxCost = 0
    costArray = []
    print("Finding mean, std deviation of cop costs")
    # Need to run through each location of robber to each object with each location of cop
    # Run through every cell in floydGrid
    for i in range(floydSize[0]): # go through robber locations
        for j in range(floydSize[1]):
            for objKey in objLocations.keys(): # go through objects
                poseGridLocY, poseGridLocX = convertPoseToGridLocation(originY, originX, mapSizeY, mapSizeX, objLocations[objKey].pose.position.y, objLocations[objKey].pose.position.x)
                path = makePath(floydWarshallNextPlace, i, j, poseGridLocY, poseGridLocX)
                for k in range(floydSize[2]):
                    for l in range(floydSize[3]):
                        cost = 0
                        for point in path:
                            poseGridLocY, poseGridLocX = point
                            pointCost = floydWarshallCosts[k][l][poseGridLocY][poseGridLocX]
                            while pointCost == np.Inf:
                                poseGridLocY+=1
                                if poseGridLocY>39:
                                    poseGridLocY = 0
                                pointCost = floydWarshallCosts[k][l][poseGridLocY][poseGridLocX]
                            cost += pointCost
                        costArray.append(cost)

    return np.mean(costArray), np.std(costArray)

def findMaxValueBased(objLocations, objNames, floydWarshallCosts, originY, originX, mapSizeY, mapSizeX): #Max is approx. 80 from file cabinet, (mean=2.7, std_dev=31.4)
    floydSize = floydWarshallCosts.shape
    maxCost = 0
    costArray = []
    print("Finding mean, std deviation of object values")
    # Need to run through each location of robber
    # Run through every cell in floydGrid
    for i in range(floydSize[0]): # go through robber locations
      for j in range(floydSize[1]):
          for objKey in objLocations.keys(): # go through objects
              poseGridLocY, poseGridLocX = convertPoseToGridLocation(originY, originX, mapSizeY, mapSizeX, objLocations[objKey].pose.position.y, objLocations[objKey].pose.position.x)
              cost = floydWarshallCosts[i][j][poseGridLocY][poseGridLocX]
              if cost != np.inf: #exclude wall locations
                  cost = (-1*cost) + objNames[objKey]
                  #rospy.loginfo(cost)
                  costArray.append(cost)
               #if we're going to normalize anyway, consider the value dimensionless in which case conversion doesn't matter
    return np.mean(costArray), np.std(costArray)



def convertPoseToGridLocation(originY, originX, mapSizeY, mapSizeX, y, x):
    y += -1*originY
    x += -1*originX
    gridLocY = int(y / mapSizeY)
    gridLocX = int(x / mapSizeX)
    return gridLocY, gridLocX

def getObjects(mapInfo):
    with open(mapInfo, 'r') as stream:
        try:
            yamled = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    originY = yamled['info']['bounds']['min_y']
    originX = yamled['info']['bounds']['min_x']

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
    return objLocations, objNames, originY, originX



if __name__ == '__main__':
    main()
