from modules.main.auv import AUV  # Import auv
import os

class Controller():
    """Controller for GUI"""

    def __init__(self):
        self.AUV = AUV()
        self.AUV.start()  # Magnet killswitch = 1

    def load_default_params(self):
        """Change current parameters back to default parameters"""

        self.AUV.config.reset_option('auv', 'tasks')
        self.AUV.config.reset_option('auv', 'motor')

    def change_params(self):
        """Opens the config file"""

        os.system('gedit config/config.ini')

    def start_auto_mode(self, value):
        """Checkbox to determine if AUV starts in auto mode when turned on"""

        self.AUV.config.set_config('auv', 'start_auto_mode', value)

    def manual_mode(self):
        """Manual mode selected (Keyboard)"""

        # task.cv_controller.stop()
        pass

    def manual_move(self, direction, power, depth):
        """Manual movement of the sub based on button press"""

        directions = {
            'brake': '`',
            'forward': 'w',
            'backward': 's',
            'strafe_l': 'q',
            'strafe_r': 'e',
            'rotate_l': 'a',
            'rotate_r': 'd',
            'up': 'r',
            'down': 'f'
        }

        # self.AUV.keyboard.navigate(directions[direction], power, depth)
        # TODO change keyboard to accept power
        pass
