import rospy
import math
import sys

import BuoyDetector
import DiceDetector
import GateDetector
from navigation import Navigation

class CVController():
    """ To decide on the task at hand than send coordinates to navigation"""

    def __init__(self):
        """ To initialize the TaskManger. """

        self.navi = Navigation()
        #TODO will be used with h_nav, r_nav and m_nav       

        self.coordinates = []
        self.dice_pair = []
        self.tasks = []
        

    def detect_gate(self):
        print('detect_gate')
        if not self.detectgate:
            self.detectgate = GateDetector.GateDetector()

        found, gate_coordinates = self.detectgate.detect()
        ''' add 'and found is True' when gate circle works '''
        if gate_coordinates[0] == 0 and gate_coordinates[1] == 0:
            if not found:
                gate_coordinates[0] = 1
            else:
                self.found_timer += 1

        if self.found_timer == 240:
            self.is_gate_found = True
            self.task_num += 1

        return found, gate_coordinates

    def detect_path(self):
        print('detect_path')
        if not self.detectpath:
            self.detectpath = PathDetector.PathDetector()
        
        found, gate_coordinates = self.detectpath()
        

    def detect_dice(self):
        print('detect_dice')
        if not self.detectdice:
            self.detectdice = DiceDetector.DiceDetector()

        '''found, dice_coordinates = self.detectdice.detect()
        if (dice_coordinates[0] == 0 and dice_coordinates[1] == 0):
            is_dice_found = True
            dice_pair = self.dicedetector.getSum()
        return found, dice_coordinates'''

        found = self.detectdice.locate_dice()

    def detect_roulette(self):
        print('detect_roulette')
        if not self.detectroulette:
            self.detectroulette = detectroulette.detect()

        found, roulette_coordinates = self.detectroulette.detect()
        if roulette_coordinates[0] == 0 and roulette_coordinates[1] == 0:
            self.is_roulette_found = True

        return found, roulette_coordinates

    def detect_cash_in(self):
        print('detect_cash_in')
        found, cash_in_coordinates = self.detectcashin.detect()
        if cash_in_coordinates[0] == 0 and cash_in_coordinates[1] == 0:
            self.is_cash_in_found = True

        return found, cash_in_coordinates

    def detect_buoy(self):
        print('detect_buoy')
        if not self.detectbuoy:
            self.detectbuoy = BuoyDetector.BuoyDetector()

        found, buoy_coordinates = self.detectbuoy.detect()
        #if buoy_coordinates[0] == 0 and buoy_coordinates[1] == 0 and found is True:
        #    self.is_buoy_found = True
        
        return found, buoy_coordinates


    def complete_gate(self):
        """ to increase the radius of the circle of the sub, we must divide """
        """ by a larger number. multiplying by a radius is not applicable """
        """ since we are only moving by -1, 0 and 1 """
        
        if (self.gate_circle_loc < 2*math.pi):
            self.gate_circle_loc += math.pi/100
            x = math.sin(self.gate_circle_loc)
            y = math.cos(self.gate_circle_loc)
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
            self.is_gate_done = True
            print('circling gate completed')
            coord_x = 0
            coord_y = 0
        
        return True, [coord_x, coord_y]

    def complete_buoy(self):
        pass

    def complete_dice(self):
        """ Will need to touch 2 dices that add up to 7 or 11 """
        if not complete_first_die:
            found, dice_direction = self.dicedetector.find_die(dice_pair[0])
        elif complete_first_die and not complete_second_die:
            found, dice_direction = self.dicedetector.find_die(dice_pair[1])
        return found, dice_direction

    def complete_roulette(self):
        pass

    def brake(self):
        pass
        #navigation.brake(self)
    
    def start(self):
        """ Starts TaskManager. """
        '''self.tasks = args

        if tasks is 'gate':
            #msg.found, coords = cv_control.detect_gate()
            pass
        elif tasks is 'path':
            pass
        elif tasks is 'dice':
            pass
        elif tasks is 'chips':
            pass
        elif tasks is 'slots':
            pass
        elif tasks is 'pinger b':
            pass
        elif tasks is 'roulette':
            pass
        elif tasks is 'pinger a':
            pass
        elif tasks is 'cash in':
            pass
        '''
        print('start cv_controller successful')
        # TODO perhaps start needs to be call along with which task you would like to perform
        #self.navigation.start()
        
    def stop(self):
        """ Stops TasksManager. """
        #rospy.on_shutdown(shutdown())
        # TODO can perhaps be used to stop a task when a error/checker is found
        pass

    def shutdown():
        """ Shutdown message for TaskManager """
        print('Shutting down Taskmanager')

cv_control = CVController()

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
    #userinput = raw_input('enter task to run...(buoy, dice, gate are only options for now)...')

    pub = rospy.Publisher('cv_to_master', CVIn)
    rospy.init_node('cv_talker', anonymous=True)
    r = rospy.Rate(30) #30hz
    
    msg = CVIn()
    
    while not rospy.is_shutdown():
    #   msg.found, coords = cv_control.detect_buoy()
    # cv_controller will need to get task from auv(houston)"""
    # which will be give to houston by the task queue """
        
        '''if not cv_control.is_gate_found:
            print('gate detect-----------------')
            msg.found, coords = cv_control.detect_gate()
        else:
            if not cv_control.is_gate_done:
                print('gate detect completed-------------')
                msg.found, coords = cv_control.complete_gate()
            else:
                print('gate task completed---------------')
                rospy.on_shutdown(close)
                break
        if not cv_control.is_buoy_found:
            msg.found, coords = cv_control.detect_buoy()
        else:
            print('buoy detect completed---------------')
            rospy.on_shutdown(close)
            break
        if not cv_control.is_dice_found:
            msg.found, coords = cv_control.detect_dice()
        else:
            msg.found, coords = cv_control.complete_dice()
        rospy.on_shutdown(close)
        break

        '''

        if cv_control.temp_task_test[cv_control.task_num] is 'gate':
            msg.found, coords = cv_control.detect_gate()
        elif cv_control.temp_task_test[cv_control.task_num] is 'buoy':
            msg.found, coords = cv_control.detect_buoy()
        elif cv_control.temp_task_test[cv_control.task_num] is 'dice':
            msg.found, coords = cv_control.detect_dice()
        elif cv_control.temp_task_test[cv_control.task_num] is 'path':
            msg.found, coords = cv_control.detect_path()
        
        msg.done = False
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