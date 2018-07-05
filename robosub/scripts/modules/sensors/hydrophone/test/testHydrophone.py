#!/usr/bin/env python
import rospy
from robosub.msg import Hydrophone
import numpy as np
import math

def get_theta(h1, h2):
    vel_of_sound = 1484 #m/s
    distance_between_hydrophones = 0.018 #meters
    try:
        theta = math.acos((vel_of_sound*(abs(h1 - h2)))/distance_between_hydrophones) * (180 / np.pi)
    except:
        return 0

    return theta

def h_status_callback(data):
    # rospy.loginfo()
    freq1 = 1.0/((abs(data.times1[0]-data.times1[1])+abs(data.times1[1]-data.times1[2])+abs(data.times1[2]-data.times1[3]))/3.0)
    freq2 = 1.0/((abs(data.times2[0]-data.times2[1])+abs(data.times2[1]-data.times2[2])+abs(data.times2[2]-data.times2[3]))/3.0)
    freq3 = 1.0/((abs(data.times3[0]-data.times3[1])+abs(data.times3[1]-data.times3[2])+abs(data.times3[2]-data.times3[3]))/3.0)
    freq4 = 1.0/((abs(data.times4[0]-data.times4[1])+abs(data.times4[1]-data.times4[2])+abs(data.times4[2]-data.times4[3]))/3.0)
    times1 = data.times1
    times2 = data.times2
    times3 = data.times3
    times4 = data.times4

    r_states = [
            'left',  # rotate left
            'staying',
            'right',  # rotate right
        ]

    infront_behind = times1[0] - times2[0]
    if infront_behind >= 0:
        #infront of auv, also includes directly left/right of auv
        left_right = times1[0] - times4[0]
        theta = get_theta(times1[0],times4[0])
        turn_degree = 90.0 - theta
        if left_right > 0:
            #left of auv
            direction = r_states[0]
        elif left_right < 0:
            #right of auv
            direction = r_states[2]
        else:
            #directly infront of auv
            direction = r_states[1]
    else:
        #behind auv
        left_right = times2[0] - times3[0]
        theta = get_theta(times2[0],times3[0])
        turn_degree = theta + 90.0
        if left_right > 0:
            #left of auv
            direction = r_states[0]
        elif left_right < 0:
            #right of auv
            direction = r_states[2]
        else:
            #directly behind auv
            direction = r_states[0]
            
    print('freq1: %.2f, freq2: %.2f, freq3: %.2f, freq4: %.2f' %(freq1,freq2,freq3,freq4))
    print('direction: %s, turn_degree: %.2f' % (direction, turn_degree))

    # rx = (abs(data.times1[0]-data.times2[0])+abs(data.times1[1]-data.times2[1])+abs(data.times1[2]-data.times2[2])+abs(data.times1[3]-data.times2[3]))/4.0
    # ry = (abs(data.times2[0]-data.times3[0])+abs(data.times2[1]-data.times3[1])+abs(data.times2[2]-data.times3[2])+abs(data.times2[3]-data.times3[3]))/4.0
    # theta = np.arctan(ry/rx)

    # print 'freq1 = {0}, freq2 = {1}, freq3 = {2}'.format(freq1, freq2, freq3)
    # print 'theta(radian) = {0}, theta(degree) = {1}'.format(theta, np.rad2deg(theta))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hydrophone_listener', anonymous=True)

    rospy.Subscriber("hydrophone_status", Hydrophone, h_status_callback)
    print('hydrophone_listener started.')
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # testData = {"times1" : [235, 195, 156, 115],"times2" : [240, 200, 160, 119],"times3" : [240, 200, 159, 118],"times4" : [0, 0, 0, 0]}

    #testData = TestData()
    #testData.times1 = [235, 195, 156, 115]
    #testData.times2 = [240, 200, 160, 119]
    #testData.times3 = [240, 200, 159, 118]
    #testData.times4 = [0, 0, 0, 0]
    #h_status_callback(testData)

    listener()

