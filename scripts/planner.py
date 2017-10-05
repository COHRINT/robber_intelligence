import rospy
import geometry_msgs
import nav_msgs
import move_base_msgs
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

def planner():

    moveStatus = ['PENDING', 'ACTIVE', 'PREEMPTED',
        'SUCCEEDED', 'ABORTED', 'REJECTED',
        'PREEMPTING', 'RECALLING', 'RECALLED',
        'LOST']

    #specific map locations
    locations = dict()
    locations["lamp"] = Pose(Point(0,0,0), Quaternion(0,0,0,0))

    # Variables to keep track of success rate, running time, and distance traveled
    n_locations = len(locations)
    n_goals = 0
    n_successes = 0
    i = n_locations
    distance_traveled = 0
    start_time = rospy.Time.now()
    running_time = 0
    location = ""
    last_location = ""

    # create a stack with python list containing destinations (append and pop)
    # when stack is empty, create new path to place with highest reward
        # in future, can improve path creation to avoid where cops are
    destinationStack = []
    current_dest = ""
    if not destinationStack:
        destinationStack.append("") # the location that has highest reward
        # create path from current location to destination then choose locations on path w/ reward
        # append these locations to stack
        

    # When robot is on the way (moveStatus = PENDING or ACTIVE), go to points on the way


    return

if __name__ == '__main__':
    try:
        planner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
