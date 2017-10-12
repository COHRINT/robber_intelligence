import rospy

def simpleGoalPub():
    rospy.init_node('simpleGoalPub')
    navGoalPub = rospy.Publisher('roy/move_base_simple/goal', mov_msgs.MoveBaseGoal, queue_size=10) # could be /move_base/current_goal
    rate = rospy.Rate(10)

    # rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'


if "__name__" == "__main__":
    try:
        simpleGoalPub()
        rospy.spin()
    except:
