#!/usr/bin/env python

'''
Robber Navigation
Escapes from cops while stealing goods
'''

__author__ = "Sousheel Vunnam"
__copyright__ = "Copyright 2018, COHRINT"
__credits__ = "Nisar Ahmed"
__license__ = "GPL"
__version__ = "1.0.0"
__maintainer__ = "Sousheel"
__email__ = "sousheel.vunnam@colorado.edu"
__status__ = ""

from planner import GoalPlanner

class robberEvasionGoalPlanner(GoalPlanner):

	def __init__(self, copName, robberName=None, robot_pose=None):

		super(robberGoalPlanner, self).__init__(robberName, robot_pose)

	def get_goal_pose(self,pose=None):
		"""Find goal pose from POMDP policy translator server

		Parameters
		----------
		Returns
		--------
		goal_pose [array]
			Goal pose in the form [x,y,theta] as [m,m,degrees]
		"""
		discrete_flag = True
		if type(self.belief) is np.ndarray:
			discrete_flag = True

		msg = None
		if discrete_flag:
			msg = DiscretePolicyTranslatorRequest()
		else:
			msg = PolicyTranslatorRequest()
		msg.name = self.robot_name
		res = None

		if discrete_flag:
			msg.belief = discrete_dehydrate(self.belief)
		else:
			if self.belief is not None:
				(msg.weights,msg.means,msg.variances) = dehydrate_msg(self.belief)
			else:
				msg.weights = []
				msg.means = []
				msg.variances = []

                print("Waiting for the POMDP Policy Service")
		rospy.wait_for_service('translator')
		try:
			pt = rospy.ServiceProxy('translator',discrete_policy_translator_service)
			res = pt(msg)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		if discrete_flag:
			self.belief = discrete_rehydrate(res.response.belief_updated,self.shapes)
		else:
			self.belief = rehydrate_msg(res.response.weights_updated,
											res.response.means_updated,
											res.response.variances_updated)

		goal_pose = list(res.response.goal_pose)

		print("NEW GOAL POSE: {}".format(goal_pose))

                # Round the given goal pose
                for i in range(len(goal_pose)):
                        goal_pose[i] = round(goal_pose[i], 2)
#                print("Rounded goal pose: {}".format(goal_pose))

		return goal_pose
