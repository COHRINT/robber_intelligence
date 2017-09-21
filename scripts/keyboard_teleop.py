import rospy
from std_msgs.msg import Float64
#from geometry_msgs.msg import Twist how do i add packages to ros

def talker():
    pub = rospy.Publisher('robber_movement', Float64, queue_size=10)
    #cmd_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5)
    rospy.init_node('keyboard_teleop')
    rate = rospy.Rate(10) # 10hz
    human_control = True #subscribe to main control node to see if keyboard teleop is activated
    #twist = Twist()
    #twist.linear.x = .1; twist.linear.y = 0; twist.linear.z = 0;
    #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
    #cmd_pub.publish(twist)
    x_vel = 0 # on keyboard left press, x = 1, right = -1
    y_vel = 0 # down = -1, up = 1
    x_ang = 0 # a = -1, d = 1
    y_ang = 0 # w = 1, s = 1
    data = 1
    while not rospy.is_shutdown():
        if data and human_control:
            rospy.loginfo(data)
            pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

