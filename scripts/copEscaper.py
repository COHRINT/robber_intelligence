#!/usr/bin/env python

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
import nav_msgs.msg as nav_msgs

class copEscaper():
    def __init(self):
        # Parameters
        self.copName = "deckard"; # get these from a yaml file later
        self.robberName = "roy"
        self.plan_tolerance = 0.1


        # 2d Nav Goal
        # ask user for a goal if needed, otherwise automatically set by endpoint
        rospy.init_node("copEscaper")
        rospy.Subscriber('/' + self.robberName + '/base_footprint', geo_msgs.Pose, robberLocationUpdate)
        rospy.Subscriber('/' + self.copName + '/base_footprint', geo_msgs.Pose, copLocationUpdate)

        rate = rospy.Rate(10)
        # rospy.on_shutdown(shutDown())

        goal = mov_msgs.MoveBaseGoal()
        goal.target_pose.header.seq = 0
        goal.target_pose.header.stamp = 0
        goal.target_pose.header.frame_id = ""

        status = ['PENDING', 'ACTIVE', 'PREEMPTED',
            'SUCCEEDED', 'ABORTED', 'REJECTED',
            'PREEMPTING', 'RECALLING', 'RECALLED',
            'LOST']

        mover_base = actionlib.SimpleActionClient("/" + self.robberName + "/move_base", mov_msgs.MoveBaseAction)
        mover_base.wait_for_server(rospy.Duration(5))

        # Keeping track
        n_locations = len(vertexes)
        i = 0

        rospy.loginfo("Starting Square Movement VROOM VROOM")

        # Go through the series of locations indefinitely
        while not rospy.is_shutdown():
            # Get cop location
            copLoc = self.copLocation

            # Get random waypoint
            i = random.randint(0, n_locations-1)
            location = vertexes[i]
            rospy.loginfo("Going to " + vertexKeys[i])

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
            else:
              rospy.loginfo("Goal failed")
              rospy.loginfo(status[state])
              mover_base.cancel_goal()
              break
            rospy.sleep(1)

    def robberLocationUpdate(self, data):
        self.robberLocation = data.data

    def copLocationUpdate(self, data):
        self.copLocation = data.data

    def detection(self, robberLoc):
        # Parameters
        range_of_vision = 5 # Degrees of field of vision of cop

        # Get current cop location
        copLoc = self.copLocation

        # Set up cop vision
        # left = tf.transformations.quaternion_from_euler(0, 0, item['orientation']-180)
        # right =

        # TODO: set up field of vision on map, if robot is in between left and right vision fields, then it is detected
        # TODO: if robot is behind wall, then it won't be caught

    def getPathClient(self, destination):
        robLoc = self.robberLocation
        tol = self.plan_tolerance

        # TODO: convert poses (destination and robber location) to PoseStamped for getPlan

        # Get plan from make_plan service
        rospy.wait_for_service("/" + self.robberName + "move_base/make_plan")
        try:
            planner = rospy.ServiceProxy("/" + self.robberName + "move_base/make_plan", nav_msgs.GetPlan)
            plan = planner(robLoc, destination, tol)
            poses = plan.plan.poses
            return poses
        except rospy.ServiceException, e:
            print "GetPlan service call failed: %s"%e
            return 0

    # def shutDown(self):
    #         rospy.loginfo("Stopping the robot...")
    #         self.move_base.cancel_goal()
    #         rospy.sleep(2)
    #         self.cmd_vel_pub.publish(Twist())
    #         rospy.sleep(1)

if __name__ == '__main__':
    try:
        copEscaper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
