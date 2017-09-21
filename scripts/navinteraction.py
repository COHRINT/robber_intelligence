import rospy
from std_msgs.msg import Float64
import geometry_msgs
import nav_msgs

def navInteraction():
    rospy.init_node('navinteraction', anonymous=True)

    # TODO: Organize what needs to be published and subscribed to into different nodes
    # TODO: Add human input for the various objectives

    # 2d Nav Goal
    # ask user for a goal if needed, otherwise automatically set by endpoint
    navGoalPub = rospy.Publisher('move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)

    # 2d Pose Estimation
    # allows user to initialize localization system used by nav stack by setting pose of robot in map
    # tells where robot is
    poseEstimationPub = rospy.Publisher('initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)

    # Static Map
    # displays map
    rospy.Subscriber("map", nav_msgs.msg.GetMap, callback)
    # rospy.Subscriber("map", nav_msgs.msg.OccupancyGrid, callback)

    # Particle Cloud
    # displays particle cloud used by localization system
    particleCloudPub = rospy.Publisher('particlecloud', geometry_msgs.msg.PoseArray, queue_size=1)

    ################
    # Cost Map
    ################
    # Robot Footprint
    # displays robot Footprint
    robotFootPub = rospy.Publisher('local_costmap/robot_footprint', geometry_msgs.msg.PolygonStamped, queue_size=1)
    # Obstacles
    # displays obstacles from costmap (robot footprint should never intersect w/ obstacles)
    obstaclePub = rospy.Publisher('local_costmap/obstacles', nav_msgs.msg.GridCells, queue_size=1)
    # Inflated Obstacles
    obstaclePub = rospy.Publisher('local_costmap/inflated_obstacles', nav_msgs.msg.GridCells, queue_size=1)
    # Unknown Space
    unknownSpacePub = rospy.Publisher('local_costmap/unknown_space', nav_msgs.msg.GridCells, queue_size=1)

    ################
    # Planning
    ################
    # Global Plan
    # displays part of global plan that local plan is pursuing
    globalPlanPub = rospy.Publisher('TrajectoryPlannerROS/global_plan', nav_msgs.msg.Path, queue_size=1)
    # Local Plan
    # displays trajectory from velocity commands commanded to base by local planner
    localPlanPub = rospy.Publisher('TrajectoryPlannerROS/local_plan', nav_msgs.msg.Path, queue_size=1)
    # Planner Plan
    # displays full plan of robot computed by global planner
    plannerPlanPub = rospy.Publisher('NavfnROS/plan', nav_msgs.msg.Path, queue_size=1)
    # Current Goal
    # displays current goal pose
    currentGoalPub = rospy.Publisher('current_goal', geometry_msgs.msg.PoseStamped, queue_size=1)


    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if data:
            rospy.loginfo(data)
            # publish all data that needs to be published
            #pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        navInteraction()
    except rospy.ROSInterruptException:
        pass

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "- I heard %s", data.data)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
