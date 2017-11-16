#!/usr/bin/env python

# Builds the costmap by evaluating where the cop is and its distance to everywhere else in the map

import yaml
import random
import math
import tf
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

from PIL import Image

def copDetection():

    # Get list of objects and their locations
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
    for item in objDict:
        itemName = item['name']
        if itemName[0:4] != "wall":
            x_loc = item['centroid_x'] + (item['width']/2 + .6) * math.cos(math.radians(item['orientation']))
            y_loc = item['centroid_y'] + (item['length']/2 + .6) * math.sin(math.radians(item['orientation']))
            quat = tf.transformations.quaternion_from_euler(0, 0, item['orientation']-180)
            itemLoc = geo_msgs.Pose(geo_msgs.Point(x_loc, y_loc, 0), geo_msgs.Quaternion(quat[0],quat[1],quat[2],quat[3]))
            objLocations[itemName] = itemLoc
    vertexes = objLocations.values()
    vertexKeys = objLocations.keys()
        
    # TODO: GetPLan not working because move base does not provide the service (EITHER FIX IT OR FIND A NEW WAY TO GET DISTANCE)
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

    copLoc = geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(1,1,0), geo_msgs.Quaternion(0,0,0,1)))
    test_path = [geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(1,1,0), geo_msgs.Quaternion(0,0,0,1))), 
        geo_msgs.PoseStamped(std_msgs.Header(), geo_msgs.Pose(geo_msgs.Point(1,1,0), geo_msgs.Quaternion(0,0,0,1)))]
    test_plans = {"kitchen": nav_msgs.Path(std_msgs.Header(), test_path)}
    print(test_plans)

    # Create costmap then sum cost of poses along path OR just calculate cost of the points then sum that up
    # max costmap area is 1 meter in front of cop
    # cost equation = (distance from max_cost point) + 1/2(how much cop is looking at that point) + (if cop is moving toward that point)
    cop_pos = copLoc.pose.position
    cop_quat = copLoc.pose.orientation
    cop_euler = tf.transformations.euler_from_quaternion([cop_quat.x, cop_quat.y, cop_quat.z, cop_quat.w])
    print (cop_euler[2])
    min_cost = 100000
    min_cost_location = ""
    for object_name, plan in test_plans:
        cost = 0
        # GO trhough each point in plan
        for point in plan:
            continue
        if cost < min_cost:
            min_cost = cost
            min_cost_location = object_name



if __name__ == '__main__':
    try:
        copDetection()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
