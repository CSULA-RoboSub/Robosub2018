import rospy
from robosub.msg import Rotation

''' created class to subscribe to IMU only once then save values'''
class GetRotation():
    ''' 
    def rotation_callback(self, rotation):
        self.rotation = rotation
        print self.rotation
        #self.rot_sub.unregister()
        rospy.signal_shutdown("only need 1 subscribe")
    '''

    def update_rot(self):
        rospy.init_node('rotation_listener', anonymous=True)
        #rospy.Subscriber('current_rotation', Rotation, self.rotation_callback)
        self.rotation = rospy.wait_for_message("current_rotation", Rotation)

    def get_yaw(self):
        return self.rotation.yaw
    
    def get_pitch(self):
        return self.rotation.pitch
    
    def get_roll(self):
        return self.rotation.yaw

    def reset(self):
        self.rotation = 0
