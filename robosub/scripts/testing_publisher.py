import rospy
from robosub.msg import Rotation

def talker():
    pub = rospy.Publisher('current_rotation', Rotation)
    rospy.init_node('rot_talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    msg = Rotation()
    count = 0
    while not rospy.is_shutdown():
        msg.yaw = count
        count += 1
        msg.pitch = count
        count += 1
        msg.roll = count
        count += 1
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass