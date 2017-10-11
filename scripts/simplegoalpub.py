import rospy

def simpleGoalPub():
    rospy.init_node('simpleGoalPub')
    navGoalPub = rospy.Publisher('roy/move_base_simple/goal', mov_msgs.MoveBaseGoal, queue_size=10) # could be /move_base/current_goal
    rate = rospy.Rate(10)


if "__name__" == "__main__":
    try:
        simpleGoalPub()
        rospy.spin()
    except:
