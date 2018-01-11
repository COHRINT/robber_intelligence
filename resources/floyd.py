#!/usr/bin/env python

"""
*************************************************
File: floyd.py
Author: Luke Burks, Sousheel Vunnam
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


def floydWarshallAlgorithm():
    # Get map information
    gridScale = .05 # size of each grid rectangle compared to map size (makes a 20x20 grid)
    curfilePath = os.path.abspath(__file__)
    curDir = os.path.abspath(os.path.join(curfilePath, os.pardir))
    parentDir = os.path.abspath(os.path.join(curDir, os.pardir))
    mapImgLocation = parentDir + "/models/map2_occupancy.png"
    mapInfoLocation = parentDir + "/models/map2_occupancy.yaml"
    mapGrid = convertMapToGrid(mapImgLocation, mapInfoLocation, gridScale)

    # Apply floyd warshall algorithm
    print(len(mapGrid))
    print(len(mapGrid[0]))
    costs, nextPlace = floyds(mapGrid)
    np.save('mapGrid', mapGrid)
    np.save('floydWarshallCosts20', costs)
    np.save('floydWarshallNextPlace', nextPlace)
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
		print(kx)

	return dist, nextPlace

def path(ux, uy, vx, vy, nextPlace):
    if nextPlace[ux, uy, vx, vy] == None:
        return []
    path = [(ux, uy)]
    while (ux != vx) and (uy != vy):
        ux, uy = nextPlace[ux, uy, vx, vy]
        path.append(u)
    return path

def displayMap(costs,pose=[0,3]):
	#lets see what the cost looks like for a position
	plt.imshow(costs[0,3]);
	plt.show();


def main():
	# #A 5x5 grid with a wall near the top
	# grid = [[0,0,0,0,0],[0,1,1,1,0],[0,0,0,1,0],[0,0,0,0,0],[0,0,0,0,0]]
    #
	# #The cops position
	# pose = [0,3];
    #
	# costs = floyds(grid);
	# displayMap(costs,pose);

    floydWarshallAlgorithm()

if __name__ == '__main__':
    main()
