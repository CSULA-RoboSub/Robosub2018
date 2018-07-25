from modules.main.auv import AUV  # Import auv
import modules.main.config as config  # Import config


class Controller():
    """ Controller for GUI"""

    def __init__(self):
        self.AUV = AUV()
        self.AUV.start()  # Magnet killswitch = 1

    def load_default_params(self):
        """ Change current parameters back to default parameters"""

        config.reset_option('auv', 'tasks')
        config.reset_option('auv', 'motor')

    def change_params(self):
        """ Opens the config file and updates the parameters"""

        self.AUV.open_config()

    def get_auto_mode_state(self):
        """ Get the start_auto_mode from config.ini"""

        return config.get_config('auv', 'start_auto_mode') == 1

    def set_auto_mode_state(self, value):
        """ Checkbox to determine if AUV starts in auto mode when turned on"""

        config.set_config('auv', 'start_auto_mode', value)

    def get_task_list(self):
        """ Get the list of tasks from config.ini"""

        return config.get_config('auv', 'tasks')

    def read_task_button(self, text):
        """ Read task string from button press"""

        print(text)
        # TODO call task in cv

    def get_color_task(self):
        """ Get the task from lower_color"""

        return config.get_config('cv', 'lower_color')[0]

    def update_color(self, task_name, range0, range1, range2):
        """ Read task name and slider range values from GUI"""

        lower_color = (
            task_name + ', ' +
            str(range0[0]) + ', ' +
            str(range1[0]) + ', ' +
            str(range2[0])
        )

        upper_color = (
            task_name + ', ' +
            str(range0[1]) + ', ' +
            str(range1[1]) + ', ' +
            str(range2[1])
        )
        config.set_config('cv', 'lower_color', lower_color)
        config.set_config('cv', 'upper_color', upper_color)
        self.AUV.update_color()

    def manual_mode(self):
        """ Manual mode selected (Keyboard)"""

        # task.cv_controller.stop()
        pass

    def manual_move(self, direction, power, rotation, depth):
        """ Manual movement of the sub based on button press"""

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
