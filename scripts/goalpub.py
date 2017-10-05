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

def goalPub():
    # 2d Nav Goal
    # ask user for a goal if needed, otherwise automatically set by endpoint
    rospy.init_node('goalPub')
    navGoalPub = rospy.Publisher('roy/move_base/goal', move_base_msgs.msg.MoveBaseGoal, queue_size=10) # could be /move_base/current_goal
    rate = rospy.Rate(10) # 10hz
    # rospy.on_shutdown(shutDown())

    goal = MoveBaseGoal()
    goal.target_pose.header.seq = 0
    goal.target_pose.header.stamp = 0
    goal.target_pose.header.frame_id = ""


    moveStatus = ['PENDING', 'ACTIVE', 'PREEMPTED',
        'SUCCEEDED', 'ABORTED', 'REJECTED',
        'PREEMPTING', 'RECALLING', 'RECALLED',
        'LOST']

    origin = Pose(Point(0,0,0), Quaternion(0,0,0,0))

    mover_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    mover_base.wait_for_server(rospy.Duration(60)) # wait 60 seconds for action server to become available

    initialpose = PoseWithCovarianceStamped()

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

     # Get the initial pose from the user
    rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
    rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
    last_location = Pose()
    rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

    # Make sure we have the initial pose
    while initial_pose.header.stamp == "":
        rospy.sleep(1)

    rospy.loginfo("Starting navigation test")

    # Begin the main loop and run through a sequence of locations
    while not rospy.is_shutdown():
        # If we've gone through the current sequence,
        # start with a new random sequence
        if i == n_locations:
            i = 0
            sequence = sample(locations, n_locations)
            # Skip over first location if it is the same as
            # the last location
            if sequence[0] == last_location:
                i = 1

        # Get the next location in the current sequence
        location = sequence[i]

        # Keep track of the distance traveled.
        # Use updated initial pose if available.
        if initial_pose.header.stamp == "":
            distance = sqrt(pow(locations[location].position.x -
                                locations[last_location].position.x, 2) +
                            pow(locations[location].position.y -
                                locations[last_location].position.y, 2))
        else:
            rospy.loginfo("Updating current pose.")
            distance = sqrt(pow(locations[location].position.x -
                                initial_pose.pose.pose.position.x, 2) +
                            pow(locations[location].position.y -
                                initial_pose.pose.pose.position.y, 2))
            initial_pose.header.stamp = ""

        # Store the last location for distance calculations
        last_location = location

        # Increment the counters
        i += 1
        n_goals += 1

        # Set up the next goal location
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = locations[location]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # Let the user know where the robot is going next
        rospy.loginfo("Going to: " + str(location))

        # Start the robot toward the next location
        self.move_base.send_goal(self.goal)

        # Allow 5 minutes to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

        # Check for success or failure
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                n_successes += 1
                distance_traveled += distance
                rospy.loginfo("State:" + str(state))
            else:
              rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

        # How long have we been running?
        running_time = rospy.Time.now() - start_time
        running_time = running_time.secs / 60.0

        # Print a summary success/failure, distance traveled and time elapsed
        rospy.loginfo("Success so far: " + str(n_successes) + "/" +
                      str(n_goals) + " = " +
                      str(100 * n_successes/n_goals) + "%")
        rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +
                      " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
        rospy.sleep(self.rest_time)

    # print("Position: ")
    # goal.target_pose.pose.position.x = input("x: ")
    # goal.target_pose.pose.position.y = input("y: ")
    # goal.target_pose.pose.position.z = 0
    #
    # print("Quaternion: ")
    # goal.target_pose.pose.orientation.x = input("x: ")
    # goal.target_pose.pose.orientation.y = input("y: ")
    # goal.target_pose.pose.orientation.z = 0
    # goal.target_pose.pose.orientation.w = 0
    #
    # while not rospy.is_shutdown():
    #     if goal:
    #         rospy.loginfo(goal)
    #         navGoalPub.publish(goal)
    #     rate.sleep()

# def shutDown(self):
#         rospy.loginfo("Stopping the robot...")
#         self.move_base.cancel_goal()
#         rospy.sleep(2)
#         self.cmd_vel_pub.publish(Twist())
#         rospy.sleep(1)

# def update_initial_pose(self, initial_pose):
#         self.initial_pose = initial_pose

if __name__ == '__main__':
    try:
        goalPub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
