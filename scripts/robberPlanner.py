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

from core.robo_tools.planner import GoalPlanner

class StationaryGoalPlanner(GoalPlanner):
        # Uses GoalPlanner's init function

	def get_goal_pose(self,pose=[0,0,0]):
		"""
                Simply returns the original position, default = [0,0,0]

                Inputs
                -------
                pose [x,y,theta] in [m,m,degrees]

		Returns
		-------
		pose  [x,y,theta] in [m,m,degrees].

		"""
		return pose
