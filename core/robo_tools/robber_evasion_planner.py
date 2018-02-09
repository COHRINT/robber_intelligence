#!/usr/bin/env python

'''
Robber Navigation
Escapes from cops while stealing goods
'''

__author__ = ["Sousheel Vunnam"]
__copyright__ = "Copyright 2018, COHRINT"
__credits__ = ["Nisar Ahmed", "Luke Barbier"]
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Sousheel"
__email__ = "sousheel.vunnam@colorado.edu"
__status__ = "Development"

from planner import GoalPlanner
import tf
import geometry_msgs.msg as geo_msgs
import math
from robber_intelligence.srv import robberEvasionGoal

class robberEvasionGoalPlanner(GoalPlanner):

	def __init__(self, copName, robberName=None, robot_pose=None):

		super(robberGoalPlanner, self).__init__(robberName, robot_pose)

	def get_goal_pose(self,pose=None):
	"""
	Find goal pose from robber evasion server
	Parameters
	----------
	Returns
	--------
	goal_pose [array]
		Goal pose in the form [x,y,theta] as [m,m,degrees]
	"""
	rospy.wait_for_service('robberEvasionGoal')
    try:
		getRobberGoal = rospy.ServiceProxy('robberEvasionGoal', robberEvasionGoal)
		goalResponse = getRobberGoal(True)
		(roll,pitch,yaw) = tf.transformations.euler_from_quaternion([goalResponse.robberGoalResponse.pose.orientation.x,
			goalResponse.robberGoalResponse.pose.orientation.y, 
			goalResponse.robberGoalResponse.pose.orientation.z, 
			goalResponse.robberGoalResponse.pose.orientation.w]
		)
		theta = math.degrees(yaw)
		goal_pose = [goalResponse.robberGoalResponse.pose.position.x, goalResponse.robberGoalResponse.pose.position.y, theta]
    except rospy.ServiceException, e:
		return pose
