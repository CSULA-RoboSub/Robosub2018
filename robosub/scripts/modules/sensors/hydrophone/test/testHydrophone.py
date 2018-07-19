#!/usr/bin/env python
import rospy
from robosub.msg import Hydrophone
import numpy as np
import math

vel_of_sound = 1484.0 #m/s
distance_between_hydrophones = 0.018 #meters

def avg_diff(t1, t2):
    avg = 0
    for i in range(1,4):
        avg += t1[i] - t2[i]
    avg /= 3
    return avg

def get_theta_acos(h1, h2):
    try:
        print("h1: %.2f h2 %.2f" %(h1,h2))
        print('inside acos:')
        print((vel_of_sound*(float(abs(avg_diff(h1 - h2))))/1000000.0) / distance_between_hydrophones)
        print('radians:')
        print(math.acos((vel_of_sound*(float(abs(avg_diff(h1 - h2))))/1000000.0) / distance_between_hydrophones))
        theta = math.acos((vel_of_sound*(float(abs(avg_diff(h1 - h2))))/1000000.0) / distance_between_hydrophones) * (180 / np.pi)
        print('degrees:')
        print(theta)
    except:
        print('acos arithmetic error')
        return False, 0

    return True, theta

def get_theta_asin(h1, h2):
    try:
        print("h1: %.2f h2 %.2f" %(h1,h2))
        print('inside asin:')
        print(distance_between_hydrophones / (vel_of_sound*(float(abs(avg_diff(h1 - h2))))/1000000.0))
        print('radians:')
        print(math.asin(distance_between_hydrophones / (vel_of_sound*(float(abs(avg_diff(h1 - h2))))/1000000.0)))
        theta = math.asin(distance_between_hydrophones / (vel_of_sound*(float(abs(avg_diff(h1 - h2))))/1000000.0)) * (180 / np.pi)
        print('degrees:')
        print(theta)
    except:
        print('asin arithmetic error')
        return False, 0

    return True, theta

def h_status_callback(data):
    # rospy.loginfo()
    times1 = data.times1
    times2 = data.times2
    times3 = data.times3
    times4 = data.times4
    freq1 = 1.0/((abs(times1[0]-times1[1])+abs(times1[1]-times1[2])+abs(times1[2]-times1[3]))/3000000.0)
    freq2 = 1.0/((abs(times2[0]-times2[1])+abs(times2[1]-times2[2])+abs(times2[2]-times2[3]))/3000000.0)
    freq3 = 1.0/((abs(times3[0]-times3[1])+abs(times3[1]-times3[2])+abs(times3[2]-times3[3]))/3000000.0)
    freq4 = 1.0/((abs(times4[0]-times4[1])+abs(times4[1]-times4[2])+abs(times4[2]-times4[3]))/3000000.0)

    r_states = [
            'left',  # rotate left
            'staying',
            'right',  # rotate right
        ]
    # slot = 1
    infront_behind = avg_diff(times1, times2)
    if infront_behind >= 0:
        #infront of auv, also includes directly left/right of auv
        left_right = avg_diff(times1, times4)
        is_good, theta = get_theta_acos(times1, times4)

        if is_good:
            turn_degree = 90.0 - abs(theta)
        else:
            is_good, theta = get_theta_asin(times1, times4)
            if is_good:
                turn_degree = abs(theta)
            else:
                print('both asin and acos arithmetic error infront auv')

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
        left_right = avg_diff(times2, times3)
        is_good, theta = get_theta_acos(times2, times3)

        if is_good:
            turn_degree = abs(theta) + 90.0
        else:
            is_good, theta = get_theta_asin(times2, times3)
            if is_good:
                turn_degree = 180 - abs(theta)
            else:
                print('both asin and acos arithmetic error behind auv')

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

