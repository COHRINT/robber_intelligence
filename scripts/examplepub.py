import rospy
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('exampletopic', Float64, queue_size=10)
    rospy.init_node('exampletalker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if data:
            rospy.loginfo(data)
            pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

