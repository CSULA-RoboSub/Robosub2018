import os
import stat
from subprocess import call
import getpass


def change_permissions(script):
    """Change the permission of the script file to user executable"""
    st = os.stat(script)
    os.chmod(script, st.st_mode | stat.S_IEXEC)


def run_script(script):
    """Run the script"""
    call(['sh', script])


def install():
    """Installs scripts for first time setup"""
    scripts = [
        'scripts/0fix_ubuntu.sh',
        'scripts/1setup_ros.sh',
        'scripts/2ros_environment.sh',
        'scripts/3ros_serial_setup.sh'
        ]

    for script in scripts:
        change_permissions(script)
        run_script(script)

    """Add user to the dialout group"""
    call(['sudo', 'adduser', getpass.getuser(), 'dialout'])

    print('\nPlease restart computer for changes to take effect.')
