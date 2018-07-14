import os
import stat
from subprocess import call
import getpass


class Installer():
    """Installs dependencies"""

    def __init__(self):
        self.scripts = [
            'scripts/0fix_ubuntu.sh',  # Fixes apt for Ubuntu 17.04
            'scripts/1setup_ros.sh',  # Installs ROS
            'scripts/2ros_environment.sh',  # Creates ROS workspace and moves code to workspace
            'scripts/3ros_serial_setup.sh',  # Installs ROS serial if not installed
            'scripts/4dependencies.sh'  # Installs dependencies
        ]

    def change_permissions(self, script):
        """Change the permission of the script file to user executable"""
        st = os.stat(script)
        os.chmod(script, st.st_mode | stat.S_IEXEC)

    def run_script(self, script):
        """Run the script"""
        call(['sh', script])

    def apt_update(self):
        """apt-get update script"""

        path = 'scripts/_apt.sh'
        self.change_permissions(path)
        self.run_script(path)

    def set_dialout(self):
        """Add user to the dialout group"""

        call(['sudo', 'adduser', getpass.getuser(), 'dialout'])

    def install(self, response=-1):
        """Installs scripts for first time setup"""

        ubuntu_fix_path = 'scripts/0fix_ubuntu.sh'

        try:
            response = int(response)
        except ValueError:
            print('Not a valid input.')
            return

        if response == -1:
            for script in self.scripts:
                self.change_permissions(script)
                self.run_script(script)

                if ubuntu_fix_path in script:
                    self.run_script(script)
        elif response >= 0 and response < len(self.scripts):
            print(self.scripts[response])
            script = self.scripts[response]
            self.change_permissions(script)
            self.apt_update()
            self.run_script(script)

        print('\nPlease restart computer for changes to take effect.')
        print('\nNew file directory will be in ~/robosub_ws')
        print('\nAfter restart please open terminal in ~/robosub_ws/ and run "catkin_make"')
