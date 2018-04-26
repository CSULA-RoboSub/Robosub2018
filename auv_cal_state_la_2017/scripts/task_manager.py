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
import math
from auv_cal_state_la_2017.msg import CVIn
from auv_cal_state_la_2017.msg import CVOut

import BuoyDetector
import DiceDetector
import GateDetector
#from modules.control.navigation import Navigation

class TaskManager:
    """ To decide on the task at hand than send coordinates to navigation"""

    def __init__(self):
        """ To initialize the TaskManger. """

        self.coordinates = []
        self.dice_pair = []
        
        self.gate_flag = False
        self.buoy_flag = False
        self.dice_flag = False
        self.roulette_flag = False
        self.dice_in_flag = False
        self.complete_first_die = False
        self.complete_second_die = False

    def detect_gate(self):
        """ When gate_detect task is called. """

        print("detect_gate")
        if not self.gate_flag:
            self.detectgate = GateDetector.GateDetector()
            self.gate_flag = True
        found, gate_coordinates = self.detectgate.detect()
        ''' add 'and found is True' when gate circle works '''
        if (gate_coordinates[0] == 0 and gate_coordinates[1] == 0 and found is True):
            global gate_found
            gate_found = True
        return found, gate_coordinates


    def detect_dice(self):
        """ When dice_detect task is called. """
        
        print("detect_dice")
        if not self.gate_flag:
            self.detectdice = DiceDetector.DiceDetector()
            self.dice_int = True

        '''found, dice_coordinates = self.detectdice.detect()
        if (dice_coordinates[0] == 0 and dice_coordinates[1] == 0):
            global dice_found
            dice_found = True
            dice_pair = self.dicedetector.getSum()
        return found, dice_coordinates'''

        found = self.detectdice.locate_dice()

    def detect_roulette(self):
        """ When roulette_detect task is called. """

        print("detect_roulette")
        if not self.roulette_flag:
            self.detectroulette = detectroulette.detect()
            self.roulette_flag = True

        found, roulette_coordinates = self.detectroulette.detect()
        if (roulette_coordinates[0] == 0 and roulette_coordinates[1] == 0):
            global roulette_found
            roulette_found = True
        return found, roulette_coordinates

    def detect_cash_in(self):
        """ When cash_in_detect task is called. """

        print("detect_cash_in")
        print("detect_cash_in")
        found, cash_in_coordinates = self.detectcashin.detect()
        if (cash_in_coordinates[0] == 0 and cash_in_coordinates[1] == 0):
            global cash_in_found
            cash_in_found = True
        return found, cash_in_coordinates

    def detect_buoy(self):
        """ When detect_buoy task is called. """
        """ Once called, will retreive coordinates from BuoyDetector """
        """ Then will publish coordinates to ROS """
        """ Navigation will retrieve coordinates from ROS """
        """ The proceed to coordinates """

        print("detect_buoy")
        if not self.buoy_flag:
            self.detectbuoy = BuoyDetector.BuoyDetector()
            self.buoy_flag = True

        found, buoy_coordinates = self.detectbuoy.detect()
        if (buoy_coordinates[0] == 0 and buoy_coordinates[1] == 0 and found is True):
            global buoy_found
            buoy_found = True
        return found, buoy_coordinates


    def complete_gate(self):
        """ Will run to complete the gate task. """
        global gate_circle_loc
        global gate_done

        """ to increase the radius of the circle of the sub, we must divide """
        """ by a larger number. multiplying by a radius is not applicable """
        """ since we are only moving by -1, 0 and 1 """
        
        if (gate_circle_loc < 2*math.pi):
            gate_circle_loc += math.pi/100
            x = math.sin(gate_circle_loc)
            y = math.cos(gate_circle_loc)
            lower_bound = -.33
            upper_bound = .33

            if (x >= upper_bound):
                coord_x = 1
            elif (x < upper_bound and x >= lower_bound):
                coord_x = 0
            elif (x < lower_bound):
                coord_x = -1

            if (y >= upper_bound):
                coord_y = 1
            elif (y < upper_bound and y >= lower_bound):
                coord_y = 0
            elif (y < lower_bound):
                coord_y = -1
        else:
            global gate_done
            gate_done = True
            print('circling gate completed')
            coord_x = 0
            coord_y = 0
        
        return True, [coord_x, coord_y]

    def complete_buoy(self):
        """ Will run to complete the buoy task """
        pass

    def complete_dice(self):
        """ Will run to complte the dice task. """
        """ Will need to touch 2 dices that add up to 7 or 11 """
        if not complete_first_die:
            found, dice_direction = self.dicedetector.find_die(dice_pair[0])
        elif (complete_first_die and not complete_second_die):
            found, dice_direction = self.dicedetector.find_die(dice_pair[1])
        return found, dice_direction

    def complete_roulette(self):
        """ Will run to complete the roulettet task. """
        pass

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
buoy_found = False
gate_found = False
dice_found = False
roulette_found = False
cash_in_found = False

gate_circle_loc = 0

gate_done = False
dice_done = False
roulette_done = False
cash_in_done = False

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

    def close():
        print('ros is shutting down')

    """ Added just to test other methods. """
    """ Will only loop one specific task until shutdown. """
    userinput = raw_input('enter task to run...(buoy, dice, gate are only options for now)')

    pub = rospy.Publisher('cv_to_master', CVIn)
    rospy.init_node('cv_talker', anonymous=True)
    r = rospy.Rate(30) #30hz
    
    msg = CVIn()
    global buoy_found
    global gate_found
    global dice_found
    global roulette_found
    global cash_in_found

    global gate_done
    global dice_done
    global roulette_done
    global cash_in_done

    while not rospy.is_shutdown():
    #   msg.found, coords = tm.detect_buoy()
    # task manager will need to get task from auv(houston)"""
    # which will be give to houston by the task queue """
        if (userinput == 'buoy'):
            if not buoy_found:
                msg.found, coords = tm.detect_buoy()
            else:
                print('buoy detect completed---------------')
                rospy.on_shutdown(close)
                break
        elif (userinput == 'dice'):
            if not dice_found:
                msg.found, coords = tm.detect_dice()
            else:
                msg.found, coords = tm.complete_dice()
        elif (userinput == 'gate'):
            if not gate_found:
                print('gate detect-----------------')
                msg.found, coords = tm.detect_gate()
            else:
                if not gate_done:
                    print('gate detect completed-------------')
                    msg.found, coords = tm.complete_gate()
                else:
                    print('gate task completed---------------')
                    rospy.on_shutdown(close)
                    break
        else:
            print('incorrect user input, please try again')
            rospy.on_shutdown(close)
            break
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

def taskcompleted():
    pass