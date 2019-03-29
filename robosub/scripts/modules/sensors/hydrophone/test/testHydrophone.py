#!/usr/bin/env python
import rospy
from robosub.msg import Hydrophone
import numpy as np
import math

vel_of_sound = 1448.9 #m/s
distance_between_hydrophones = 0.018 #meters

def avg_diff(t1, t2):
    avg = 0
    for i in range(1,4):
        avg += t1[i] - t2[i]
    avg /= 3
    return avg

def diff_for_all(h1, h2):
    ret = []

    for i in range(0, 4):
        ret.append(h1[i] - h2[i])

    return ret

def get_theta_acos(h1, h2):
    # print("h1: %.2f, h2 %.2f" %(h1,h2))
    print('h1, h2')
    print("%.2f, %.2f" %(h1,h2))
    # print('h1 - h2: {}'.format(h1-h2))
    print('h1 - h2')
    print('{}'.format(h1-h2))
    # print('inside acos: %.2f' % ((vel_of_sound*(float(abs(avg_diff(h1 - h2))))/1000000.0) / distance_between_hydrophones))
    try:
        rad = math.acos((vel_of_sound*(float(abs(avg_diff(h1 - h2))))/1000000.0) / distance_between_hydrophones)
        deg = math.acos((vel_of_sound*(float(abs(avg_diff(h1 - h2))))/1000000.0) / distance_between_hydrophones) * (180 / np.pi)
        
        # print('radians: %.2f' % (rad))
        # print('degrees: %.2f' % (th))
        print('radians')
        print('%.2f' % (rad))
        print('deg')
        print('%.2f' % (th))
        theta = deg
    except:
        # print('acos arithmetic error')
        print('radians')
        print('Error')
        print('deg')
        print('Error')
        theta = None
    return theta

# def get_theta_asin(h1, h2):
#     try:
#         print("h1: %.2f h2 %.2f" %(h1,h2))
#         print('inside asin:')
#         print(distance_between_hydrophones / (vel_of_sound*(float(abs(avg_diff(h1 - h2))))/1000000.0))
#         print('radians:')
#         print(math.asin(distance_between_hydrophones / (vel_of_sound*(float(abs(avg_diff(h1 - h2))))/1000000.0)))
#         theta = math.asin(distance_between_hydrophones / (vel_of_sound*(float(abs(avg_diff(h1 - h2))))/1000000.0)) * (180 / np.pi)
#         print('degrees:')
#         print(theta)
#     except:
#         print('asin arithmetic error')
#         return False, 0

#     return True, theta

def h_status_callback(data):
    # rospy.loginfo()
    times1 = data.times1
    times2 = data.times2
    times3 = data.times3
    times4 = data.times4
    freq1 = []
    freq2 = []
    freq3 = []
    freq4 = []
    print('times1')
    print('{}, {}, {}, {}'.format(times1[0], times1[1], times1[2], times1[3]))
    
    print('times2')
    print('{}, {}, {}, {}'.format(times2[0], times2[1], times2[2], times2[3]))
    print('times3')
    print('{}, {}, {}, {}'.format(times3[0], times3[1], times3[2], times3[3]))
    print('times4')
    print('{}, {}, {}, {}'.format(times4[0], times4[1], times4[2], times4[3]))
    for x in range(0,3):
        freq1.append(1.0/((abs(times1[x]-times1[x+1]))/100000.0))
        freq2.append(1.0/((abs(times2[x]-times2[x+1]))/100000.0))    
        freq3.append(1.0/((abs(times3[x]-times3[x+1]))/100000.0))
        freq4.append(1.0/((abs(times4[x]-times4[x+1]))/100000.0))
        print('freq1, freq2, freq3, freq4')
        print('{:.2f}, {:.2f}, {:.2f}, {:.2f}'.format(freq1[x], freq2[x], freq3[x], freq4[x]))
        
    r_states = [
            'left',  # rotate left
            'staying',
            'right',  # rotate right
        ]
    # slot = 1
    # infront_behind = diff_for_all(times1, times2)
    turn_degree = []
    direction = []
    theta = []
    infront_behind = []
    left_right = []
    for i in range(0,4):
        infront_behind.append(times1[i] - times2[i])
        if infront_behind[i] >= 0:
            #infront of auv, also includes directly left/right of auv
            left_right.append(times1[i] - times4[i])
            theta.append(get_theta_acos(times1[i], times4[i]))

            # if is_good:
            if theta[i] is not None:
                turn_degree.append(90.0 - abs(theta[i]))
            else:
                turn_degree.append(None)
            # else:
            #     is_good, theta = get_theta_asin(times1, times4)
            #     if is_good:
            #         turn_degree = abs(theta)
            #     else:
            #         print('both asin and acos arithmetic error infront auv')
            #         turn_degree = 0

            if left_right > 0:
                #left of auv
                direction.append(r_states[0])
            elif left_right < 0:
                #right of auv
                direction.append(r_states[2])
            else:
                #directly infront of auv
                direction.append(r_states[1])
        else:
            #behind auv
            left_right.append(times2[i] - times3[i])
            theta.append(get_theta_acos(times2[i], times3[i]))

            # if is_good:
            #     turn_degree = abs(theta) + 90.0

            if theta[i] is not None:
                turn_degree.append(abs(theta[i]) + 90.0)
            else:
                turn_degree.append(None)
            # else:
            #     is_good, theta = get_theta_asin(times2, times3)
            #     if is_good:
            #         turn_degree = 180 - abs(theta)
            #     else:
            #         print('both asin and acos arithmetic error behind auv')
            #         turn_degree = 0

            if left_right > 0:
                #left of auv
                direction.append(r_states[0])
            elif left_right < 0:
                #right of auv
                direction.append(r_states[2])
            else:
                #directly behind auv
                direction.append(r_states[0])
    
    # print('freq1: %.2f, freq2: %.2f, freq3: %.2f, freq4: %.2f' %(freq1,freq2,freq3,freq4))
    # print('direction: {}, turn_degree: {}\n'.format(direction, turn_degree))
    # print('------------------------------------------------------')

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

