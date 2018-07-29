"""Import line-oriented command interpreter"""
import cmd
import subprocess
import time
import os
from modules.main.auv import AUV  # Import auv
import modules.main.status as status  # Import status logger


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

        if arg.lower() == 'on' or arg == '4':
            AUV.motor.toggle_state(MOTOR_ON)
        elif arg.lower() == 'off' or arg == '5':
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

    # lower/upper color ###############################################################################################
    def do_lower(self, arg):
        '\n[task] 0-255,0-255,0-255\
         \nused to change lower of color filter'

        if not arg.lower() == '':
            try:
                arg1, arg2 = parse_color(arg)
                AUV.houston.cvcontroller.set_lower_color(arg1, arg2)
            except:
                print('\nincorrect input.\
                       \n[task] 0-255,0-255,0-255')
        else:
            print('\n[task] 0-255,0-255,0-255\
                   \nused to change lower of color filter')

    def do_upper(self, arg):
        '\n[task] 0-255,0-255,0-255\
         \nused to change upper of color filter'

        if not arg.lower() == '':
            try:
                arg1, arg2 = parse_color(arg)
                AUV.houston.cvcontroller.set_upper_color(arg1, arg2)
            except:
                print('\nincorrect input.\
                       \n[task] 0-255,0-255,0-255')
        else:
            print('\n[task] 0-255,0-255,0-255\
                   \nused to change lower of upper filter')

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

    # auto-complete navigation
    def complete_navigation(self, text, line, start_index, end_index):
        args = ['cv', 'keyboard']

        if text:
            return [arg for arg in args if arg.startswith(text)]
        else:
            return args

    # task #######################################################################################################
    def do_task(self, arg):
        '\nto start please enter\
         \n[task] <0-10>\
         \nstop task by entering [task]\
         \n'

        print('type: [task ?] to see all options')

        if arg.lower() == 'stop' or arg.lower() == '':
            AUV.stop_task()
        elif arg.lower() == 'all':
            AUV.perform_tasks()
        elif arg.lower() == 'heading' or arg.lower() == 'h':
            AUV.save_heading()
        elif arg == '?':
            print('\nto start please enter:\
                   \n[task] (0-{})\
                   \nstop task by entering [task] or [task stop]\
                   \nrun all tasks by entering [task all]\
                   \nsave current heading by entering [task heading]\
                   \n'.format(len(AUV.houston.tasks)))

            AUV.display_tasks()
        elif not arg == '':
            try:
                arg = int(arg)
            except:
                print '\nINVALID NUMBER INPUT'
                pass

            if arg >= 0 and arg <= len(AUV.houston.tasks):
                AUV.specific_task(arg)
                # AUV.display_tasks()
        else:
            print('\nto start please enter:\
                   \n[task] (0-{})\
                   \nstop task by entering [task] or [task stop]\
                   \nrun all tasks by entering [task all]\
                   \nsave current heading by entering [task heading]\
                   \n'.format(len(AUV.houston.tasks)))

            AUV.display_tasks()

    # config ###########################################################################################################
    def do_config(self, arg):
        '\nOpens the config file and updates the parameters'

        AUV.open_config()

    # status logger ####################################################################################################
    def do_logging(self, arg):
        '\n[on/off] or [1/0] Turn on or off status logging\
         \n[toggle] to toggle logging\
         \n[state] or no argument to print current state'

        if arg.lower() == 'on' or arg == '1':
            status.is_logging = True
            # AUV.status_logger.toggle_logging(1)
        elif arg.lower() == 'off' or arg == '0':
            status.is_logging = False
            # AUV.status_logger.toggle_logging(0)
        # elif arg.lower() == 'toggle':
            # AUV.status_logger.toggle_logging()
        else:
            print('\nstatus logging state: %d' % status.is_logging)

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

        AUV.stop_task()
        AUV.stop()
        print('Closing Robosub')

        return True


def parse_color(arg):
    arg1, arg2 = arg.split()
    list = []
    temp_list = arg2.split(',')
    for i in temp_list:
        if int(i) < 0:
            i = 0
        elif int(i) > 255:
            i = 255
        list.append(int(i))
    return arg1, list


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
