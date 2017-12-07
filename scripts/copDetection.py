#!/usr/bin/env python

# Builds the costmap by evaluating where the cop is and its distance to everywhere else in the map

import yaml
import random
import math
import tf
import numpy as np
import matplotlib.pyplot as plt
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


def copDetection():

    copName = "deckard"
    robberName = "roy"

    # Get list of objects and their locations
    mapInfo = 'map2.yaml'
    objLocations = getObjects(mapInfo)
    vertexes = objLocations.values()
    vertexKeys = objLocations.keys()

    # TODO: GetPlan not working because move base does not provide the service (EITHER FIX IT OR FIND A NEW WAY TO GET DISTANCE)
    # tol = .1
    # robberName = "roy"

    # robLoc = geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(1,1,0), geo_msgs.Quaternion(0,0,0,1)))
    # destination = geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(0,1,0), geo_msgs.Quaternion(0,0,0,1)))

    # print(robLoc)
    # print(destination)

    # # Get plan from make_plan service
    # #rospy.wait_for_service("/" + robberName + "move_base/make_plan")
    # try:
    #     planner = rospy.ServiceProxy("/" + robberName + "/move_base/make_plan", nav_srv.GetPlan)
    #     plan = planner(robLoc, destination, tol)
    #     poses = plan.plan.poses
    #     print(poses)
    # except rospy.ServiceException, e:
    #     print "GetPlan service call failed: %s"%e
    #     return 0

    # rospy.Subscriber("/" + copName + "/", geo_msgs.Pose, getCopLocation)
    # rospy.Subscriber("/" + robberName + "/", geo_msgs.Pose, getRobberLocation)
    copLoc = geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(1,1,0), geo_msgs.Quaternion(0,0,1,0)))
    pastCopLoc = geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(0,1,0), geo_msgs.Quaternion(0,0,1,0)))
    robLoc = geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(1,0,0), geo_msgs.Quaternion(0,0,1,0)))
    test_path = [geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(0,1,0), geo_msgs.Quaternion(0,0,0,1))),
        geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(1,0,0), geo_msgs.Quaternion(0,0,0,1)))]
    test_plans = {"kitchen": nav_msgs.Path(std_msgs.Header(), test_path)}

    curCost, curDestination = chooseDestination(test_plans, copLoc, robLoc, pastCopLoc)
    print("Minimum Cost: " + str(curCost) + " at " + curDestination)
    #move base to curDestination
    # state = mover_base.get_state()
    # pathFailure = False
    # while (state!=3 and pathFailure==False): # SUCCESSFUL
    #     #wait a second rospy.sleep(1)
    #     newCost = evaluatePath(test_plans[curDestination])
    #     if newCost > curCost*2:
    #         pathFailure = True
    # rospy.loginfo("Just Reached " + vertexKeys[i])


    # Floyd warshall stuff
    mapGrid = np.load('mapGrid.npy')
    floydWarshallCosts = np.load('floydWarshallCosts.npy')
    evaluateFloydCost(copLoc, robLoc, floydWarshallCosts, mapGrid)
    # plt.imshow(mapGrid, interpolation='nearest')
    # plt.show()




def evaluateFloydCost(copLoc, pose, floydWarshallCosts, mapGrid):
    copGridLocY, copGridLocX = convertPoseToGridLocation(copLoc.pose.position.y , copLoc.pose.position.x, mapGrid)
    poseGridLocY, poseGridLocX = convertPoseToGridLocation(pose.pose.position.y, pose.pose.position.x, mapGrid)
    return floydWarshallCosts[copGridLocY][copGridLocX][poseGridLocY][poseGridLocX])
    # plt.imshow(floydWarshallCosts[copGridLocY][copGridLocX], interpolation='nearest')
    # plt.show()

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
            itemLoc = geo_msgs.Pose(geo_msgs.Point(x_loc, y_loc, 0), geo_msgs.Quaternion(quat[0],quat[1],quat[2],quat[3]))
            objLocations[itemName] = itemLoc
    return objLocations

def chooseDestination(plans, copLoc, robLoc, pastCopLoc):
    # Create costmap then sum cost of poses along path OR just calculate cost of the points then sum that up
    # cost equation = (distance from max_cost point) + 1/2(how much cop is looking at that point) + (if cop is moving toward that point)
    cop_pos = copLoc.pose.position
    cop_quat = copLoc.pose.orientation
    cop_euler = tf.transformations.euler_from_quaternion([cop_quat.x, cop_quat.y, cop_quat.z, cop_quat.w])

    # max costmap area is .5 meter in front of cop
    max_cost_distance = .5
    max_cost_pos = geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(cop_pos.x + max_cost_distance*math.cos(cop_euler[0]), cop_pos.y + max_cost_distance*math.sin(cop_euler[1]), 0), geo_msgs.Quaternion(0,0,0,1)))

    min_cost = 100000
    min_cost_location = ""

    # Evaluate each object's plan
    for object_name, plan in plans.items():
        pathCost = evaluatePath(plan, copLoc, robLoc, pastCopLoc, max_cost_pos)
        if pathCost < min_cost:
            min_cost = pathCost
            min_cost_location = object_name
        # print(pathCost)
    return min_cost, min_cost_location

def evaluatePath(plan, copLoc, robLoc, pastCopLoc, max_cost_pos):
    pathCost = 0
    # Evaluate each point in plan
    for point in plan.poses:
        pointCost = euclideanDistance(point, max_cost_pos) + angleCost(copLoc, point) + velocityCost(pastCopLoc, copLoc, robLoc)
        # print(pointCost)
        pathCost += pointCost
    return pathCost

# Takes in two PoseStamped points and returns the distance between them
def euclideanDistance(point1, point2):
    xSqr = (point2.pose.position.x - point1.pose.position.x)**2
    ySqr = (point2.pose.position.y - point1.pose.position.y)**2
    zSqr = (point2.pose.position.z - point1.pose.position.z)**2
    distance = math.sqrt(xSqr + ySqr + zSqr)
    return distance

# Takes in the cop's location and a point (PoseStamped) and returns the cost based on if the cop is facing that point
def angleCost(copLoc, point):
    cop_quat = copLoc.pose.orientation
    cop_euler = tf.transformations.euler_from_quaternion([cop_quat.x, cop_quat.y, cop_quat.z, cop_quat.w])

    #print(cop_euler[2])
    copVec = [math.cos(cop_euler[2]), math.sin(cop_euler[2])]
    copToPointVec = [copLoc.pose.position.x - point.pose.position.x, copLoc.pose.position.y - point.pose.position.y]
    #print (copToPointVec)
    angle = angleSimilarity(copVec, copToPointVec)
    # how can i map this angle to a score (the larger the angle, the smaller the score)
    # score = scipy.stats.norm(1.57, .2).cdf(angle)
    return 0

# compare euclidean distance (and angle) between pastLoc and curLoc to point
def velocityCost(pastCopLoc, curCopLoc, point):
    distWeight = .5
    angWeight = .5
    # velocityVec = [curCopLoc.pose.position.x - pastCopLoc.pose.position.x, curCopLoc.pose.position.y - pastCopLoc.pose.position.y, curCopLoc.pose.position.z - pastCopLoc.pose.position.z]
    # angle = angleSimilarity(velocity, point)
    dist = euclideanDistance(pastCopLoc, point) - euclideanDistance(curCopLoc, point) # If robot is closer than it was previously, then cost is positive
    ang = angleCost(pastCopLoc, point) - angleCost(curCopLoc, point)
    velCost = distWeight * dist + angWeight * ang
    print(velCost)
    return velCost

def angleSimilarity(vec1, vec2):
    unitVec1 = vec1 / np.linalg.norm(vec1)
    unitVec2 = vec2 / np.linalg.norm(vec2)
    angle = np.arccos(np.clip(np.dot(unitVec1, unitVec2), -1.0, 1.0))
    return angle



def getCopLocation(data):
    copLoc = data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def getRobberLocation(data):
    robLoc = data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


if __name__ == '__main__':
    try:
        copDetection()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
