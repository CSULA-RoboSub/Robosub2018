from scripts import setup_ros

response = raw_input(
                '\nAre you sure you want to do first time setup for robosub? [y/n]: '
            ).lower()
if response.lower() == 'y':
    print('Setting up dependencies for Ubuntu 17.04')
    setup_ros.install()
