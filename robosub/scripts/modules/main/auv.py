import rospy
from std_msgs.msg import Int8

# from test import test_movement
from config.config import Config
from modules.control.motor import Motor
from modules.control.navigation import Navigation
from modules.control.keyboard import Keyboard
from modules.main.status_logger import StatusLogger

# try:
#     from houston import Houston
# except ValueError:q
#     print('Required hardware not detected.')
from houston import Houston


class AUV():
    """AUV Master, automates tasks"""

    def __init__(self):
        rospy.init_node('AUV', anonymous=True)  # initialize AUV rosnode

        rospy.Subscriber('kill_switch', Int8, self.kill_switch_callback)  # Subscriber for magnet kill switch

        self.motor_state = None
        self.tasks = None

        # self.test
        self.config = Config()  # initialize Config() class
        self.read_config()

        self.motor = Motor(self.motor_state)  # initialize Motor() class
        self.navigation = Navigation()  # initialize Navigation() class
        self.keyboard = Keyboard(self.navigation)  # initialize Keyboard() class
        self.status_logger = StatusLogger()  # initialize StatusLogger() class

        # try:
        self.houston = Houston(self.navigation) # initialize Houston() class
        # except NameError:
        #     print('Houston is not initialized.')

    def kill_switch_callback(self, data):
        if data.data == 1:
            self.start()
        if data.data == 0:
            self.stop()

    def read_config(self):
        """ Reads from config/config.ini"""

        self.motor_state = self.config.get_config('auv', 'motor_state')  # read motor state from config
        self.tasks = self.config.get_config('auv', 'tasks')  # read tasks from config
        # TODO send kevin the tasks list whenever it is read/changed

    def keyboard_nav(self):
        """Navigate the robosub using keyboard controls"""

        self.keyboard.getch()

    def perform_tasks(self):
        """Has houston perform task"""
        # try:
        self.houston.start_all_tasks()
        # except AttributeError:
        #     print('houston not initialized')
    
    def specific_task(self, task_num):
        self.houston.do_one_task(task_num)

    def stop_task(self):
        """Has houston stop task"""
        self.houston.stop_task()

    # def stop_one_task(self, task_num):
    #     self.houston.stop_one_task(task_num)

    def start(self):
        """Starts the modules when magnet killswitch is plugged in"""

        self.motor.start()
        self.navigation.start()
        self.keyboard.start()
        self.status_logger.start()
        # try:
        self.houston.start()
        # except AttributeError:
        #     print('houston not initialized')
        # self.cv.start(self.tasks)

    def stop(self):
        """Stops the modules when magnet killswitch is removed"""

        self.motor.stop()
        self.navigation.stop()
        self.keyboard.stop()
        self.status_logger.stop()
        # try:
        self.houston.stop()
        # except AttributeError:
        #     print('houston not initialized')
