import rospy
import math

# TODO create following msgs for subscriber and publisher
#from robosub.msg import Task
#from robosub.msg import CVData

from modules.sensors.computer_vision import GateDetector
from modules.sensors.computer_vision import BuoyDetector
from modules.sensors.computer_vision import DiceDetector


class CVController():
    
    def __init__(self):
        # follow code for video capturing
        self.cap = cv2.VideoCapture(0)
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('video_output/gate-' + str(time.time()) + '_output.avi', self.fourcc, 20.0, (640, 480))

        # create instance of classes
        self.gatedetector = GateDetector()
        self.buoydetector = BuoyDetector()
        self.dicedetector = DiceDetector()

        rospy.Subscriber('houston_to_cv', Task, HoustonCallback)
        rospy.pub_cv_data = rospy.Publisher('cv_to_houston', CVData)
        rospy.init_node('CV_talker', anonymous=True)
        

    def HoustonCallback(data):
        rospy.loginfo(rospy.get_caller_id() + "testing: %s", data.data)
    
    # TODO implement methods for cv_controller
    #def get_coordinates?