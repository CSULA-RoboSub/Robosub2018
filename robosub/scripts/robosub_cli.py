"""Import line-oriented command interpreter"""
import cmd
import subprocess
import time
import os
from modules.main.auv import AUV  # Import auv


class CLI(cmd.Cmd):
    """AUV command line interpreter"""

    intro = '\nType help or ? to list commands.'
    prompt = 'auv> '

    # test #############################################################################################################
    def do_test(self, arg):
        '\n[movement] to test movement\
         \n[cv] to test cv direction finder'

        # if arg.lower() == 'movement':
        #     test_movement.main()

        # TODO finish test

    # auto-complete test
    def complete_test(self, text, line, start_index, end_index):
        args = ['movement', 'cv']

        if text:
            return [arg for arg in args if arg.startswith(text)]
        else:
            return args

    # tasks ############################################################################################################
    def do_tasks(self, arg):
        '\n[view] to view tasks\
         \n[set] to set tasks list\
         \n[reset] to reset tasks list'

        if arg.lower() == 'view':
            print(AUV.tasks)
        elif arg.lower() == 'set':
            # AUV.config.set_config('auv', 'tasks', response)
            # AUV.read_config()
            # TODO set tasks
            # AUV.set_config('tasks', '0 1 2 3 4 5 6 7 8')
            print('test')
        elif arg.lower() == 'reset':
            AUV.config.reset_option('auv', 'tasks')
        else:
            print(AUV.tasks)

        # TODO make a config file for default tasks

    # auto-complete tasks
    def complete_tasks(self, text, line, start_index, end_index):
        args = ['view', 'set', 'reset']

        if text:
            return [arg for arg in args if arg.startswith(text)]
        else:
            return args

    # motor ############################################################################################################
    def do_motor(self, arg):
        '\n[on/off] or [1/0] Turn on or off motors\
         \n[toggle] to toggle the current state\
         \n[state] or no argument to print current state'

        if arg.lower() == 'on' or arg == '1':
            AUV.motor.toggle_state(1)
        elif arg.lower() == 'off' or arg == '0':
            AUV.motor.toggle_state(0)
        elif arg.lower() == 'toggle':
            AUV.motor.toggle_state()
        else:
            print('\nmotor state: %d' % AUV.motor.get_state())

    # auto-complete motor
    def complete_motor(self, text, line, start_index, end_index):
        args = ['on', 'off', 'toggle', 'state']

        if text:
            return [arg for arg in args if arg.startswith(text)]
        else:
            return args

    # navigation #######################################################################################################
    def do_navigation(self, arg):
        '\n[cv] toggle computer vision task manager\
         \n[keyboard] keyboard manual navigation'

        if arg.lower() == 'cv' or arg.lower() == 'tm':
            # TODO cv taskmanager
            print(arg)
        elif arg.lower() == 'keyboard' or arg.lower() == 'kb':
            AUV.keyboard_nav()
        # elif len(arg.split()) == 4:
        #     AUV.navigation.navigate(*parse(arg))
        else:
            print(
                '\n[cv] toggle computer vision task manager\
                 \n[keyboard] keyboard manual navigation'
            )

    # auto-complete navigation
    def complete_navigation(self, text, line, start_index, end_index):
        args = ['cv', 'keyboard']

        if text:
            return [arg for arg in args if arg.startswith(text)]
        else:
            return args

    # CV model picker ##################################################################################################
    def do_model(self, arg):
        '\n[0/1/2/3/4] Change to a specific CV model\
         \n[view] view current CV model'

        if arg >= 0 and arg <= 4:
            AUV.model_picker.change_model(arg)
        else:
            AUV.model_picker.get_model()

    # auto-complete CV model picker
    def complete_model(self, text, line, start_index, end_index):
        args = ['view']

        if text:
            return [arg for arg in args if arg.startswith(text)]
        else:
            return args

    # status logger ####################################################################################################
    def do_logging(self, arg):
        '\n[on/off] or [1/0] Turn on or off status logging\
         \n[toggle] to toggle logging\
         \n[state] or no argument to print current state'

        if arg.lower() == 'on' or arg == '1':
            AUV.status_logger.toggle_logging(1)
        elif arg.lower() == 'off' or arg == '0':
            AUV.status_logger.toggle_logging(0)
        elif arg.lower() == 'toggle':
            AUV.status_logger.toggle_logging()
        else:
            print('\nstatus logging state: %d' % AUV.status_logger.is_logging)

    # auto-complete status logger
    def complete_logging(self, text, line, start_index, end_index):
        args = ['on', 'off', 'toggle', 'state']

        if text:
            return [arg for arg in args if arg.startswith(text)]
        else:
            return args

    # exit #############################################################################################################
    def do_exit(self, arg):
        '\nExits auv'

        AUV.stop()
        print('Closing Robosub')

        return True


def parse(arg):
    'Convert a series of zero or more numbers to an argument tuple'
    return tuple(map(int, arg.split()))


if __name__ == '__main__':
    # open roscore in subprocess
    print('Setting up roscore.')
    os.system('killall -9 roscore')
    os.system('killall -9 rosmaster')
    os.system('killall -9 rosout')
    roscore = subprocess.Popen('roscore')
    time.sleep(1)

    AUV = AUV()  # initialize AUV() class

    print('\n***Plug in magnet after setting up configurations to start AUV.***')
    print('\n***Set motor state to 1 to start motors.***')

    AUV.start()  # TESTING PURPOSES ONLY. REMOVE AFTER TESTING (simulates magnet killswitch = 1#########################

    CLI().cmdloop()  # run AUV command interpreter

    # close roscore and rosmaster on exit
    subprocess.Popen.kill(roscore)
    os.system('killall -9 rosmaster')
    os.system('killall -9 rosout')
