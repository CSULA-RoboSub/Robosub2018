from modules.main.auv import AUV  # Import auv


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

        pass