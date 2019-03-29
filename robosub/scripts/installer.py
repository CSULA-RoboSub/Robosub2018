from installation_scripts.setup_ros import Installer


def main():
    installer = Installer()

    response = raw_input(
                    '\nAre you sure you want to do first time setup for robosub? [y/n]: '
                ).lower()
    if response == 'y':
        print('Setting up dependencies for Ubuntu 18.04')
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
                    \n4: dependencies\
                    \n5: set_dialout\
                    \n>> '
                )

        try:
            if int(selection) >= -1 and int(selection) <= 4:
                installer.install(selection)
            elif int(selection) == 5:
                installer.set_dialout()
        except ValueError:
            print('Not a valid input.')


if __name__ == "__main__":
    main()
