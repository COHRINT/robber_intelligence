#!/usr/bin/env python
"""
Goal planner module, subclass GoalPlanner and implemenent find_goal_pose() method
Returns the new pose given the current pose

Subclasses should contain logic on when to change the goal pose ie, when it reaches the original goal pose or dynamically each update loop change the goal pose
"""
__author__ = ["LT"]
__copyright__ = "Copyright 2017, Cohrint"
__credits__ = ["Matthew Aitken", "Nick Sweet", "Nisar Ahmed"]
__license__ = "GPL"
__version__ = "3.0.0"
__maintainer__ = "Luke Barbier"
__email__ = "luke.barbier@colorado.edu"
__status__ = "Development"

from pdb import set_trace

import tf
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from abc import ABCMeta, abstractmethod

class GoalPlanner(object):
    """The GoalPlanner Super Class generates goal poses for a robot.

    Publishes goals to the rostopic: /robot_name/move_base_simple/goal
    """
    reached_pose_proximity = 0.5 # distance until a pose is reached
    
    __metaclass__ = ABCMeta

    def __init__(self, robot_name=None, robot_pose=None):
        """
        Initializes a goalPlanner object
        
        Parameters
        ---------
        robot_pose : simply to initialize self.prev_goal_pose for when update() gets called
        robot_name : name of the robot

        """
        if robot_name is None:
            print("No robot_name given")
            print("Check the instantiation line of GoalPlanner")
            raise
        elif robot_pose is None:
            print("No robot_pose given")
            print("Check the instantiation line of GoalPlanner")
            raise

        self.robot_name = robot_name.lower()
        # Make a publisher for Goals to be communicated on updating
        goal_pose_topic = '/' + self.robot_name + '/move_base_simple/goal'
        self.pub = rospy.Publisher(goal_pose_topic, PoseStamped, queue_size=10)

        # Used to determine whether to create a new rosmsg in the GoalPlanner.update() method
        self.prev_goal_pose = robot_pose
        self.prev_pose = robot_pose

    def reached_pose(self, pose, goal_pose):
        if len(pose) != 3 or len(goal_pose) != 3:
            print("Improper parameters to reached_pose()")
            raise
        # Check if the robot is stopped
        if self.prev_pose != pose:
            return False

        # Check if we've reached the pose also
        x = abs(pose[0] - goal_pose[0])
        y = abs(pose[1] - goal_pose[1])
        if x < self.reached_pose_proximity and y < self.reached_pose_proximity:
            return True
        else:
            return False
        
    @abstractmethod
    def get_goal_pose(self,pose=None):
        """Returns a goal pose using the subclasser's get_goal_pose method

        Parameters
        ----------
        pose : list [x,y,theta] in [m,m,degrees].
        Returns
        -------
        goal_pose : list [x,y,theta] in [m,m,degrees].

        """
        pass
        
    def update(self,pose=None):
        """
        Updates the goal pose of the robot
        Parameters
        ---------
        positions [x,y, degrees] floats
        """
        
#        print("ENTERING GOAL PLANNER UDPATE")

        old_goal_pose = self.prev_goal_pose # 1st time = None
            
        new_goal_pose = self.get_goal_pose(pose)

        print("goal_pose: " +str(new_goal_pose))

        # Check if it's actually a new pose before we publish
        if self.is_new_pose(old_goal_pose, new_goal_pose) is True:
            self.create_ROS_goal_message(new_goal_pose)

        # record the new goal pose for the next time around
        self.prev_goal_pose = new_goal_pose
        self.prev_pose = pose
        
#        set_trace()

    def is_new_pose(self,old_pose,new_pose):
        """
        Checks if the old_pose and new_pose are roughly equal

        Returns 
        -------
        True : poses different
        False : poses roughly equal
        """
        
        if old_pose is not None and new_pose is not None:
            if new_pose[0] != old_pose[0] or new_pose[1] != old_pose[1]:
#                print("New Goal pose")
                return True
            else:
#                print("Same Goal pose")
                return False
        else:
#            print("New pose")
            return True
        

    def create_ROS_goal_message(self, goal_pose=None):
        """
        creates and publishes a ros goal msg, given the goal_pose as a list
        Parameters
        ---------
        goal_pose : [x,y, orientation] in [m,m,radians]
        """
        print("New goal msg being created.")
        rospy.sleep(1)
        
        if goal_pose is None:
            print("No goal pose given to the create_ROS_goal_message function")
            print("Check planner.py")
            raise
        
        move_base_goal = PoseStamped()

        theta = np.rad2deg(goal_pose[2])
        quaternions = tf.transformations.quaternion_from_euler(0, 0, theta)
        move_base_goal.pose.position.x = goal_pose[0]
        move_base_goal.pose.position.y = goal_pose[1]
        move_base_goal.pose.orientation.x = quaternions[0]
        move_base_goal.pose.orientation.y = quaternions[1]
        move_base_goal.pose.orientation.z = quaternions[2]
        move_base_goal.pose.orientation.w = quaternions[3]
        move_base_goal.header.frame_id = '/map'
        move_base_goal.header.stamp = rospy.Time.now()
        self.pub.publish(move_base_goal)
        
    def stop(self, pose=[0,0,0]):
        create_ROS_goal_message(pose)
