import rospy
from modules.sensors.imu.gather_rotation import GetRotation


def testing():
    
    getrotation = GetRotation()
    
    while True:
        getrotation.update_rot()
        yaw = getrotation.get_yaw()
        pitch = getrotation.get_pitch()
        roll = getrotation.get_roll()
        print 'here is the yaw: {}'.format(yaw)
        print 'here is the pitch: {}'.format(pitch)
        print 'here is the roll: {}'.format(roll)

if __name__ == '__main__':
    testing()