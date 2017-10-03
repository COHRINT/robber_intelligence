import rospy
import geometry_msgs
import nav_msgs
import move_base_msgs

def goalPub():
    # 2d Nav Goal
    # ask user for a goal if needed, otherwise automatically set by endpoint
    rospy.init_node('goalPub')
    navGoalPub = rospy.Publisher('roy/move_base/goal', move_base_msgs.msg.MoveBaseGoal, queue_size=10) # could be /move_base/current_goal
    rate = rospy.Rate(10) # 10hz

    goal = MoveBaseGoal()
    goal.target_pose.header.seq = 0
    goal.target_pose.header.stamp = 0
    goal.target_pose.header.frame_id = ""

    print("Position: ")
    goal.target_pose.pose.position.x = input("x: ")
    goal.target_pose.pose.position.y = input("y: ")
    goal.target_pose.pose.position.z = 0

    print("Quaternion: ")
    goal.target_pose.pose.orientation.x = input("x: ")
    goal.target_pose.pose.orientation.y = input("y: ")
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 0

    while not rospy.is_shutdown():
        if goal:
            rospy.loginfo(goal)
            navGoalPub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        goalPub()
    except rospy.ROSInterruptException:
        pass
