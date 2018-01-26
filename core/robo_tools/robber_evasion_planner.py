#!/usr/bin/env python

'''
Robber Navigation
Escapes from cops while stealing goods
'''

__author__ = ["Sousheel Vunnam", "Jamison McGinley"]
__copyright__ = "Copyright 2018, COHRINT"
__credits__ = ["Nisar Ahmed", "Luke Barbier"]
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Sousheel"
__email__ = "sousheel.vunnam@colorado.edu"
__status__ = ""

from planner import GoalPlanner
import tf
import geometry_msgs.msg as geo_msgs

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
		(roll,pitch,yaw) = tf.transformations.euler_from_quaternion([goalResponse.pose.orientation.x, \
        	goalResponse.pose.orientation.y, goalResponse.pose.orientation.z, goalResponse.pose.orientation.w])
		# is this in radians or degrees?
		theta = yaw
		goal_pose = [goalResponse.pose.position.x, goalResponse.pose.position.y, theta]
		return goal_pose
    except rospy.ServiceException, e:
		return [0, 0, 0]
