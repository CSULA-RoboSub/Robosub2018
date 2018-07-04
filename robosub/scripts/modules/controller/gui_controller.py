from modules.main.auv import AUV  # Import auv
import os

class Controller():
    """Controller for GUI"""

    def __init__(self):
        self.AUV = AUV()
        self.AUV.start  # Magnet killswitch = 1

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

    def auto_mode(self):
        """Auto mode selected (CV)"""

        print('auto mode')
        pass

    def manual_mode(self):
        """Manual mode selected (Keyboard)"""

        print('manual mode')
        pass
