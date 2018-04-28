from scripts.setup_ros import Installer


def main():
    installer = Installer()

    response = raw_input(
                    '\nAre you sure you want to do first time setup for robosub? [y/n]: '
                ).lower()
    if response == 'y':
        print('Setting up dependencies for Ubuntu 17.04')
        installer.install()
        installer.set_dialout()
    elif response == 'n':
        selection = raw_input(
                    '\nInput the number to install the script:\
                    \n-1: all\
                    \n0: fix_ubuntu\
                    \n1: setup_ros\
                    \n2: ros_environment\
                    \n3: ros_serial_setup\
                    \n4: set_dialout\
                    \n>> '
                )
        if int(selection) >= -1 and int(selection) <= 3:
            installer.install(selection)
        elif int(selection) == 4:
            installer.set_dialout()


if __name__ == "__main__":
    main()