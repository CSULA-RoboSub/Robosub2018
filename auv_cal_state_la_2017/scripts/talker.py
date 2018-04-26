#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from auv_cal_state_la_2017.msg import CVIn
from auv_cal_state_la_2017.msg import CVOut

import BuoyDetector
import DiceDetector
#from modules.control.navigation import Navigation

class TaskManager:
    """ To decide on the task at hand than send coordinates to navigation"""

    def __init__(self):
        """ To initialize the TaskManger. """

        self.coordinates = []
        self.detectbuoy = BuoyDetector.BuoyDetector()
        # self.detectdice = DiceDetector.DiceDetector()
        #self.navigation = Navigation()
        #self.is_killswitch_on = navigation.check_kill()

    def detect_gate(self):
        """ When gate_detect task is called. """

    # TODO need to find out which cv node to get coordinates from
        print("detect_gate")
        #self.coordinates.append(randint(-1,1))
        #self.coordinates.append(randint(-1,1))
        #self.navigation.nagivate(1, 1, 1, 1)

    def detect_dice(self):
        """ When dice_detect task is called. """
        """ calls DiceDetector module and creates an instance """
        """ then gets the coordinates of the dice in the pool """
        """ putting coordinates in dice_coordinates """
        """ sends coordinates to navigation module """

        print("detect_dice")
        #self.coordinates.append(randint(-1,1))
        #self.coordinates.append(randint(-1,1))
        dice_coordinates = self.detectdice.locate_dice()
        #self.navigation.nagivate(dice_coordinates)

    def detect_roulette(self):
        """ When roulette_detect task is called. """

        print("detect_roulette")
        #self.coordinates.append(randint(-1,1))
        #self.coordinates.append(randint(-1,1))
        #self.navigation.nagivate(1, 1, 1, 1)

    def detect_cash_in(self):
        """ When cash_in_detect task is called. """

        print("detect_cash_in")
        #self.coordinates.append(randint(-1,1))
        #self.coordinates.append(randint(-1,1))
        #self.navigation.nagivate(1, 1, 1, 1)

    def detect_buoy(self):
        """ When detect_buoy task is called. """
        """ Once called, will retreive coordinates from ROS with subscriber """
        """ Then will send coordinates to Navigation module """
        """ Sub will navigate towards listed coordinates """

        # included 2 mothods to broadcast coordinates
        # could broadcast through ROS or be called by the module name

        """ This is the portion that must be sent back to HOUSTON """
        """ We are calling DetectBuoy to detect the buoy in the pool """
        """ Then sending the coordinates back to HOUSTON """
        """ which will be sent from HOUSTON to NAVIGATION """
        """ ********************************************* """
        print("detect_buoy")
        found, buoy_coordinates = self.detectbuoy.detect()

        # TODO send coordinates to Houston
        return found, buoy_coordinates


        #self.navigation.nagivate(buoy_coordinates)

        '''pub_task = rospy.Publisher('coordinates', Int32MultiArray, queue_size=10)
        pub_array = Int32MultiArray(data=buoy_coordinates)

        rospy.loginfo('sending coordinates to ROS')
        pub_task.publish(pub_array)

        rospy.init_node('buoy_coordinates', anonymous=True)
        rospy.Subscriber('xy_coordinate', Int32MultiArray, directions)'''

    def direction(data):
        """ Directions is called upon after rospy.subscriber to obtain value from ROS """

        rospy.loginfo('Receiving coordinates from buoy_coordinates')
        coordinates = data.data
        #self.navigation.navigate(coordinates)

    def brake(self):
        """ When brake task is called by auv.py. """

        #navigation.brake(self)
    
    def start(self):
        """ Starts TaskManager. """

        # TODO perhaps start needs to be call along with which task you would like to perform
        #self.navigation.start()
        
    def stop(self):
        """ Stops TasksManager. """
        rospy.on_shutdown(shutdown())
        # TODO can perhaps be used to stop a task when a error/checker is found
        pass

    def shutdown():
        """ Shutdown message for TaskManager """

        print('Shutting down Taskmanager')

tm = TaskManager()

def talker():
    # pub = rospy.Publisher('cv_to_master', CVIn)
    # rospy.init_node('custom_talker', anonymous=True)
    # r = rospy.Rate(30) #30hz
    # msg = CVIn()
    # msg.found = 1
    # msg.horizontal = -1
    # msg.vertical = -1
    # msg.distance = 1.25
    # msg.targetType = 1.0

    # while not rospy.is_shutdown():
    #     rospy.loginfo(msg)
    #     pub.publish(msg)
    #     r.sleep()

    pub = rospy.Publisher('cv_to_master', CVIn)
    rospy.init_node('cv_talker', anonymous=True)
    r = rospy.Rate(30) #30hz
    
    msg = CVIn()

    while not rospy.is_shutdown():

        msg.found, coords = tm.detect_buoy()
        msg.horizontal = coords[0]
        msg.vertical = coords[1]
        msg.distance = 1.25
        msg.targetType = 1.0

        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: 
        pass