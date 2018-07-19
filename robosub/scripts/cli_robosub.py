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

    # tasks ############################################################################################################
    def do_tasks(self, arg):
        '\n[view] to view tasks\
         \n[reset] to reset tasks list'

        if arg.lower() == 'view':
            print(AUV.tasks)
            print('test')
        elif arg.lower() == 'reset':
            AUV.config.reset_option('auv', 'tasks')
        else:
            print(AUV.tasks)

    # auto-complete tasks
    def complete_tasks(self, text, line, start_index, end_index):
        args = ['view', 'set', 'reset']

        if text:
            return [arg for arg in args if arg.startswith(text)]
        else:
            return args

    # motor ############################################################################################################
    def do_motor(self, arg):
        '\n[on/off] Turn on or off motors\
         \n[toggle] to toggle the current state\
         \n[state] or no argument to print current state'

        MOTOR_ON = 4
        MOTOR_OFF = 5

        if arg.lower() == 'on':
            AUV.motor.toggle_state(MOTOR_ON)
        elif arg.lower() == 'off':
            AUV.motor.toggle_state(MOTOR_OFF)
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
        '\n[cv] toggle computer vision (start/1 or stop/0)\
         \n[keyboard] keyboard manual navigation'

        if arg.lower() == 'cv start' or arg.lower() == 'cv 1':
            # temp method for testing purposes
            AUV.perform_tasks()
            # print(arg)
        elif arg.lower() == 'cv stop' or arg.lower() == 'cv 0':
            AUV.stop_task()
            # print(arg)
        elif arg.lower() == 'keyboard' or arg.lower() == 'kb':
            AUV.keyboard_nav()
        else:
            print('\n[cv] toggle computer vision (start/1 or stop/0)\
                   \n[keyboard] keyboard manual navigation')

    # task #######################################################################################################
    def do_task(self, arg):
        '\nto start please enter\
         \n[task] <0-10>\
         \nstop task by entering [task]\
         \n'

        AUV.display_tasks()

        if arg.lower() == 'stop' or arg.lower() == '':
            AUV.stop_task()
        elif not arg == '':
            try:
                arg = int(arg)
            except:
                print '\nINVALID NUMBER INPUT'
                pass

        if arg >= 0 and arg <= 10:
            AUV.specific_task(arg)
            # AUV.display_tasks()
        else:
            print('\nto start please enter:\
                   \n[task] (0-10)\
                   \nstop task by entering [task]\
                   \n')
        
    # auto-complete navigation
    def complete_navigation(self, text, line, start_index, end_index):
        args = ['cv', 'keyboard']

        if text:
            return [arg for arg in args if arg.startswith(text)]
        else:
            return args

    # config ###########################################################################################################
    def do_config(self, arg):
        '\nOpens the config file and updates the parameters'

        AUV.update_config()

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
    """Convert a series of zero or more numbers to an argument tuple"""
    return tuple(map(int, arg.split()))


def start_roscore():
    """Check if roscore is running. If not starts roscore"""

    name = 'roscore'
    ps = os.popen('ps -Af').read()

    if name not in ps:
        # open roscore in subprocess
        print('Setting up roscore.')
        os.system('killall -9 roscore')
        os.system('killall -9 rosmaster')
        os.system('killall -9 rosout')

        roscore = subprocess.Popen('roscore')
        time.sleep(1)
        return roscore

    return False


if __name__ == '__main__':
    roscore = start_roscore()

    AUV = AUV()  # initialize AUV() class

    print('\n***Plug in magnet after setting up configurations to start AUV.***')
    print('\n***Set motor state to on (4) to start motors.***')

    AUV.start()  # TESTING PURPOSES ONLY. REMOVE AFTER TESTING (simulates magnet killswitch = 1 ########################

    CLI().cmdloop()  # run AUV command interpreter

    # close roscore and rosmaster on exit if opened by CLI
    if(roscore):
        subprocess.Popen.kill(roscore)
        os.system('killall -9 rosmaster')
        os.system('killall -9 rosout')
