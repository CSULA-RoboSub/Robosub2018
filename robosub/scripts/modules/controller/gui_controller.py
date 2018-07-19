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
        """Opens the config file and updates the parameters"""

        self.AUV.update_config()

    def start_auto_mode(self, value):
        """Checkbox to determine if AUV starts in auto mode when turned on"""

        self.AUV.config.set_config('auv', 'start_auto_mode', value)

    def manual_mode(self):
        """Manual mode selected (Keyboard)"""

        # task.cv_controller.stop()
        pass

    def manual_move(self, direction, power, rotation, depth):
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

        if any(x in directions[direction] for x in ['w', 'q', 'e', 's']):
            self.AUV.keyboard.set_power(m_power=power)
        elif any(x in directions[direction] for x in ['a', 'd']):
            self.AUV.keyboard.set_power(r_power=power)
        elif any(x in directions[direction] for x in ['r', 'f']):
            self.AUV.keyboard.set_power(h_power=power)

        self.AUV.keyboard.navigate(directions[direction], rotation, depth)
