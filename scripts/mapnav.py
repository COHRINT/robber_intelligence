#!/usr/bin/env python

''' Main Robber Intelligence Map Navigation '''

import rospy
import geometry_msgs
import nav_msgs
import move_base_msgs
import actionlib
import actionlib_msgs.msg as act_msgs # import *
import geometry_msgs.msg as geo_msgs # geo_msgs.Pose, geo_msgs.PoseWithCovarianceStamped, geo_msgs.Point, geo_msgs.Quaternion, geo_msgs.Twist, geo_msgs.Vector3
import move_base_msgs.msg as mov_msgs # import mov_msgs.MoveBaseAction, mov_msgs.MoveBaseGoal

class mapNav():
    def __init__(self):
        rospy.init_node('mapNav')
        self.velPub = rospy.Publisher('deckard/cmd_vel_mux/input/', geo_msgs.Twist, queue_size=5)
        self.mover_base = actionlib.SimpleActionClient("deckard/move_base", mov_msgs.MoveBaseAction)
        #self.mover_base.wait_for_server(rospy.Duration(5))
        rospy.on_shutdown(self.turnOff)

        self.roomDict = {"study": geo_msgs.Pose(geo_msgs.Point(-1,1,0), geo_msgs.Quaternion(0,0,0,1)), "hallway": geo_msgs.Pose(geo_msgs.Point(1,-1,0), geo_msgs.Quaternion(0,0,0,1)),
            "library": geo_msgs.Pose(geo_msgs.Point(1,1,0), geo_msgs.Quaternion(0, 0,0,1)), "billiard room":geo_msgs.Pose(geo_msgs.Point(-1,-1,0), geo_msgs.Quaternion(0,0,0,1))}
        self.objectDict = []
        self.roomKeys = self.roomDict.keys()
        self.roomVals = self.roomDict.values()
        self.status = ['PENDING', 'ACTIVE', 'PREEMPTED',
            'SUCCEEDED', 'ABORTED', 'REJECTED',
            'PREEMPTING', 'RECALLING', 'RECALLED',
            'LOST']

        planner = ""
        while (planner.lower()is not "human" or planner.lower() is not "auto"):
           planner = raw_input("Would you like to move based on human input or automatically?: Please type \"human\" or \"auto\": ")
           print(planner.lower())
        
        # Run
        self.goalPub(planner.lower())
        
        # Cancel all goals and stop movement
        self.turnOff()

    def goalPub(self, planner):
        # 2d Nav Goal
        # ask user for a goal if needed, otherwise automatically set by endpoint
        num_locations = len(self.roomDict)
        i = 0

        rospy.loginfo("Starting Movement VROOM VROOM")

        while not rospy.is_shutdown():
            # Get next location
            if (planner=="auto"):
                if i == num_locations:
                    i = 0
                next_location = self.roomVals[i]
                i += 1
            elif (planner=="human"):
                userInput = raw_input("Where would you like to go?: ")
                while userInput not in self.roomKeys:
                    userInput = raw_input("Please enter in a valid location: ")
                next_location = self.roomDict[userInput]

            # Send the goal to the robot
            goal = mov_msgs.MoveBaseGoal()
            goal.target_pose.pose = next_location
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            self.mover_base.send_goal(goal)
            rospy.loginfo("Travelling to " + roomKeys[i])

            # Wait until robot travels to location or time out after 3 minutes
            mover_base.wait_for_result(rospy.Duration(180))

            state = self.mover_base.get_state()
            if state == 3: #SUCCESSFUL
                rospy.loginfo(i)
                rospy.loginfo("Just Reached " + self.roomKeys[i])
            else:
              rospy.loginfo("Goal failed: " + self.status[state])
              break
            rospy.sleep(1)


    def turnOff(self):
        rospy.loginfo("Turning Off BEEP BOOP")
        self.mover_base.cancel_goal()
        rospy.loginfo("bruh")
        self.velPub.publish(geo_msgs.Twist())

if __name__ == '__main__':
    try:
        mapNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
