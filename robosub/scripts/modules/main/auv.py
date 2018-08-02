import rospy
import os
from std_msgs.msg import Int8

import modules.main.config as config
from modules.control.motor import Motor
from modules.control.navigation import Navigation
from modules.control.keyboard import Keyboard
import modules.servos.dropper as dropper
import modules.servos.torpedo as torpedo
import modules.main.status as status

from houston import Houston


class AUV():
    """AUV Master, automates tasks"""

    def __init__(self):
        rospy.init_node('AUV', anonymous=True)  # initialize AUV rosnode

        rospy.Subscriber('kill_switch', Int8, self.kill_switch_callback)  # Subscriber for magnet kill switch

        self.motor_state = None
        self.tasks = None

        self.navigation = Navigation()  # initialize Navigation() class
        self.houston = Houston(self.navigation, self.tasks)  # initialize Houston() class

        self.update_config()  # read parameters from the config.ini file

        self.motor = Motor(self.motor_state)  # initialize Motor() class
        self.keyboard = Keyboard(self.navigation)  # initialize Keyboard() class

    def kill_switch_callback(self, data):
        if data.data == 1:
            self.start()
        if data.data == 0:
            self.stop()

    # CONFIG ##############################################################################################
    def update_config(self):
        """ Loads the updated parameters from config"""

        self.motor_state = config.get_config('auv', 'motor_state')  # read motor state from config
        self.tasks = config.get_config('auv', 'tasks')  # read tasks from config
        self.houston.cvcontroller.update_config()

    def reset_config_option(self, section=None, option=None):
        """ Resets the config to default configurations given selection and option"""

        config.reset_option(section, option)
        # TODO make better

    def open_config(self):
        """ Opens the config file and updates the parameters"""

        os.system('gedit config/config.ini')
        self.update_config()

    # def update_config(self):
    #     """ Loads the updated parameters from config"""

    #     self.read_config()
    #     self.houston.cvcontroller.set_model()  # read and set all models from config

    # def update_color(self):
    #     """ Update RGB/HSV for computer vision from config"""

    #     lower_color = config.get_config('cv', 'lower_color')
    #     upper_color = config.get_config('cv', 'upper_color')
    #     self.houston.cvcontroller.set_lower_color(lower_color[0], lower_color[1:])
    #     self.houston.cvcontroller.set_upper_color(upper_color[0], upper_color[1:])

    # def read_config(self):
    #     """ Reads from config/config.ini"""

    #     self.motor_state = config.get_config('auv', 'motor_state')  # read motor state from config
    #     self.tasks = config.get_config('auv', 'tasks')  # read tasks from config

    def keyboard_nav(self):
        """Navigate the robosub using keyboard controls"""

        self.keyboard.getch()

    def perform_tasks(self):
        """Has houston perform task"""

        self.houston.start_task('all', None)

    def specific_task(self, task_num):
        """Has houston do specific task"""

        self.houston.start_task('one', task_num)

    def specific_task_for_gui(self, task_name):
        """Has houston do specific task from gui"""

        self.houston.start_task_from_gui('one', task_name)

    def stop_task(self):
        """Has houston stop task"""

        self.houston.stop_task()

    def display_tasks(self):
        """Has houston display list of tasks"""

        self.houston.print_tasks()

    # def stop_one_task(self, task_num):
    #     self.houston.stop_one_task(task_num)

    def start(self):
        """Starts the modules when magnet killswitch is plugged in"""

        self.motor.start()
        self.navigation.start()
        self.keyboard.start()
        self.houston.start()

    def stop(self):
        """Stops the modules when magnet killswitch is removed"""

        self.motor.stop()
        self.navigation.stop()
        self.keyboard.stop()
        status.stop()
        self.houston.stop()

    def save_heading(self):
        self.navigation.save_current_heading()

    def prime_torpedo(self, side):
        if side.lower() == 'left' or side.lower() == 'right':
            torpedo.prime_torpedo(side)
        else:
            print('Invalid side input. Please enter correct side.')

    def fire_torpedo(self, side):
        if side.lower() == 'left' or side.lower() == 'right':
            torpedo.fire_torpedo(side)
        else:
            print('Invalid side input. Please enter correct side.')

    def dropper_state(self, state):
        dropper.drop_control(state)
