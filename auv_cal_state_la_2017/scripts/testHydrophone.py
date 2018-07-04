#!/usr/bin/env python
import rospy
from auv_cal_state_la_2017.msg import Hydrophone
import numpy as np

class TestData:
    def __init__(self):
        pass

def hStatusCallback(data):
    # rospy.loginfo()
    freq1 = 1.0/((abs(data.times1[0]-data.times1[1])+abs(data.times1[1]-data.times1[2])+abs(data.times1[2]-data.times1[3]))/3.0)
    freq2 = 1.0/((abs(data.times2[0]-data.times2[1])+abs(data.times2[1]-data.times2[2])+abs(data.times2[2]-data.times2[3]))/3.0)
    freq3 = 1.0/((abs(data.times3[0]-data.times3[1])+abs(data.times3[1]-data.times3[2])+abs(data.times3[2]-data.times3[3]))/3.0)
    freq4 = 1.0/((abs(data.times4[0]-data.times4[1])+abs(data.times4[1]-data.times4[2])+abs(data.times4[2]-data.times4[3]))/3.0)
    times1 = data.times1
    times2 = data.times2
    times3 = data.times3
    times4 = data.times4
    # rx = (abs(data.times1[0]-data.times2[0])+abs(data.times1[1]-data.times2[1])+abs(data.times1[2]-data.times2[2])+abs(data.times1[3]-data.times2[3]))/4.0
    # ry = (abs(data.times2[0]-data.times3[0])+abs(data.times2[1]-data.times3[1])+abs(data.times2[2]-data.times3[2])+abs(data.times2[3]-data.times3[3]))/4.0
    # theta = np.arctan(ry/rx)

    # print 'freq1 = {0}, freq2 = {1}, freq3 = {2}'.format(freq1, freq2, freq3)
    # print 'theta(radian) = {0}, theta(degree) = {1}'.format(theta, np.rad2deg(theta))
    vSound = 1484 #m/s
    d = 0.018 #meters
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hydrophoneListener', anonymous=True)

    rospy.Subscriber("hydrophone_status", Hydrophone, hStatusCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # testData = {"times1" : [235, 195, 156, 115],"times2" : [240, 200, 160, 119],"times3" : [240, 200, 159, 118],"times4" : [0, 0, 0, 0]}

    #testData = TestData()
    #testData.times1 = [235, 195, 156, 115]
    #testData.times2 = [240, 200, 160, 119]
    #testData.times3 = [240, 200, 159, 118]
    #testData.times4 = [0, 0, 0, 0]
    #hStatusCallback(testData)

    listener()

