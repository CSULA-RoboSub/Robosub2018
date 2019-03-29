import rospy
from ez_async_data.msg import Rotation
# from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher('current_rotation', Rotation, queue_size=0)
    # pubRoll = rospy.Publisher('roll', Float32, queue_size=0)
    # pubPitch = rospy.Publisher('pitch', Float32, queue_size=0)
    # pubYaw = rospy.Publisher('yaw', Float32, queue_size=0)

    rospy.init_node('ez_test_pub', anonymous=True)
    r = rospy.Rate(30)

    msg = Rotation()

    # msgRoll = Float32()
    # msgPitch = Float32()
    # msgYaw = Float32()

    while not rospy.is_shutdown():

        msg.pitch = 0
        msg.roll = 0
        msg.yaw = 

        # rospy.loginfo(msg)
        pub.publish(msg)

        # msgRoll.data = 0
        # msgPitch.data = 0
        # msgYaw.data = 50

        # pubRoll.publish(msgRoll)
        # pubPitch.publish(msgPitch)
        # pubYaw.publish(msgYaw)

        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
