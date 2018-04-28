import rospy
import ConfigParser
from shutil import copyfile
from std_msgs.msg import Int8

# from test import test_movement
from modules.control.motor import Motor
from modules.control.navigation import Navigation
from modules.control.keyboard import Keyboard
from modules.main.status_logger import StatusLogger


class AUV():
    """AUV Master, automates tasks"""

    def __init__(self):
        rospy.init_node('AUV', anonymous=True)  # initialize AUV rosnode

        rospy.Subscriber('kill_switch', Int8, self.kill_switch_callback)  # Subscriber for magnet kill switch

        self.motor_state = 0
        self.tasks = []

        self.get_config()

        # self.test
        self.motor = Motor(self.motor_state)  # initialize Motor() class
        self.navigation = Navigation()  # initialize Navigation() class
        self.keyboard = Keyboard()  # initialize Keyboard() class
        self.status_logger = StatusLogger()  # initialize StatusLogger() class
        # TODO self.cv = CV() # initialize CV() class

    def kill_switch_callback(self, data):
        if data.data == 1:
            self.start()
        if data.data == 0:
            self.stop()

    def get_config(self):
        """Reads variables from config/config.ini file.
        Creates config.ini file from template_config.ini file if config.ini does not exist
        """

        config = ConfigParser.RawConfigParser()
        config_file_path = 'config/config.ini'

        try:
            config.readfp(open(config_file_path))
        except IOError:
            print('setting up config.ini file.')
            copyfile('config/template_config.ini', 'config/config.ini')
            config.readfp(open(config_file_path))

        if config.has_option('auv_config', 'motor_state'):
            self.motor_state = config.getint('auv_config', 'motor_state')
            print('motor state: %d' % self.motor_state)

        if config.has_option('auv_config', 'tasks'):
            self.tasks = config.get('auv_config', 'tasks').split(', ')

            if not self.tasks and config.has_option('auv_config', 'default_tasks'):
                self.tasks = config.get('auv_config', 'default_tasks').split(', ')

            print('tasks: %s' % self.tasks)

    def set_config(self, var, value, is_reset=False):
        """Assign given value to respective var in config/config.ini or reset to default values"""

        config = ConfigParser.RawConfigParser()
        config_file_path = 'config/config.ini'

        try:
            config.readfp(open(config_file_path))
        except IOError:
            print('setting up config.ini file.')
            copyfile('config/template_config.ini', 'config/config.ini')
            config.readfp(open(config_file_path))

        if var == 'tasks' and is_reset:
            config.set('auv_config', 'tasks', config.get('auv_config', 'default_tasks'))
            self.tasks = config.get('auv_config', 'default_tasks').split()
            print('tasks: %s' % self.tasks)
        elif var == 'tasks' and not is_reset:
            # TODO set tasks
            value.split()

        with open(config_file_path, 'wb') as configfile:
            config.write(configfile)

    def keyboard_nav(self):
        """Navigate the robosub using keyboard controls"""

        self.keyboard.getch()

    def start(self):
        """Starts the modules when magnet killswitch is plugged in"""

        self.motor.start()
        self.navigation.start()
        self.keyboard.start()
        self.status_logger.start()
        # self.cv.start(self.tasks)

    def stop(self):
        """Stops the modules when magnet killswitch is removed"""

        self.motor.stop()
        self.navigation.stop()
        self.keyboard.stop()
        self.status_logger.stop()
