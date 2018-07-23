import sys
import subprocess
import time
import os
from functools import partial
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QColorDialog
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QColor

from modules.controller.gui_controller import Controller


class App(QWidget):

    def __init__(self):
        super(App, self).__init__()
        self.controller = Controller()  # Initialize gui controller
        self.title = 'Robosub GUI'
        self.left = 10
        self.top = 10
        self.width = 1432
        self.height = 850
        self.initUI()

    def initUI(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        self.main_layout = QtWidgets.QHBoxLayout(self)
        self.left_section()
        self.right_section()

        self.show()

    def left_section(self):

        # Layouts ##########
        self.left_layout = QtWidgets.QVBoxLayout()
        self.display_layout = QtWidgets.QHBoxLayout()
        self.camera_layout = QtWidgets.QHBoxLayout()

        # Widgets ##########

        # display_layout
        self.display0 = QtWidgets.QGraphicsView(self)  # display0
        self.display_layout.addWidget(self.display0)
        self.display1 = QtWidgets.QGraphicsView(self)  # display1
        self.display_layout.addWidget(self.display1)

        self.left_layout.addLayout(self.display_layout)

        # camera_layout
        self.btn_camera0 = QtWidgets.QPushButton('Camera 0', self)  # btn_camera0
        self.camera_layout.addWidget(self.btn_camera0)
        self.btn_camera1 = QtWidgets.QPushButton('Camera 1', self)  # btn_camera1
        self.camera_layout.addWidget(self.btn_camera1)
        self.btn_camera2 = QtWidgets.QPushButton('Camera 2', self)  # btn_camera2
        self.camera_layout.addWidget(self.btn_camera2)

        self.left_layout.addLayout(self.camera_layout)

        # sensor_data
        self.sensor_data = QtWidgets.QTableView(self)  # sensor_data
        self.sensor_data.setShowGrid(True)
        # sensor_data.setRowCount(4)
        # sensor_data.setColumnCount(7)
        self.sensor_data.horizontalHeader().setVisible(False)
        self.sensor_data.horizontalHeader().setHighlightSections(False)
        self.sensor_data.verticalHeader().setVisible(False)
        self.sensor_data.verticalHeader().setHighlightSections(False)
        self.left_layout.addWidget(self.sensor_data)
        self.messages = QtWidgets.QTextBrowser(self)  # messages
        self.left_layout.addWidget(self.messages)

        self.main_layout.addLayout(self.left_layout)

        self.left_connections()  # Controller connctions

    def right_section(self):

        # Layouts ##########
        self.right_layout = QtWidgets.QVBoxLayout()
        self.mode_layout = QtWidgets.QHBoxLayout()

        # Auto Mode Tab
        self.task_selection = QtWidgets.QWidget()  # task_selection
        self.task_layout = QtWidgets.QVBoxLayout(self.task_selection)

        # Manual Mode Tab
        self.manual_controls = QtWidgets.QWidget()  # manual_controls
        self.controls_layout = QtWidgets.QGridLayout(self.manual_controls)
        self.controls_layout.setContentsMargins(0, 0, 0, 0)
        self.controls_layout.setHorizontalSpacing(4)
        self.controls_layout.setVerticalSpacing(3)
        self.power_layout = QtWidgets.QVBoxLayout()
        self.rotation_layout = QtWidgets.QVBoxLayout()
        self.depth_layout = QtWidgets.QVBoxLayout()

        # Computer Vision Tab
        self.computer_vision = QtWidgets.QWidget()  # computer_vision
        self.cv_layout = QtWidgets.QGridLayout(self.computer_vision)
        self.cv_layout.setContentsMargins(0, 0, 0, 0)
        self.cv_layout.setHorizontalSpacing(2)
        self.cv_layout.setVerticalSpacing(3)
        # TODO inner layout of cv tab

        # Widgets ##########

        # mode_layout
        self.btn_load_default = QtWidgets.QPushButton('Load Default Params', self)  # btn_load_default
        self.mode_layout.addWidget(self.btn_load_default)
        self.btn_change_params = QtWidgets.QPushButton('Change Params', self)  # btn_change_params
        self.mode_layout.addWidget(self.btn_change_params)
        self.chk_autostart = QtWidgets.QCheckBox('Start in Auto Mode', self)  # chk_autostart
        self.chk_autostart.setChecked(self.controller.get_auto_mode_state())
        self.mode_layout.addWidget(self.chk_autostart)

        self.right_layout.addLayout(self.mode_layout)

        # tab_widget
        self.tab_widget = QtWidgets.QTabWidget(self)  # tab_widget
        self.tab_widget.addTab(self.task_selection, 'Auto Mode')
        self.tab_widget.addTab(self.manual_controls, 'Manual Mode')
        self.tab_widget.addTab(self.computer_vision, 'Computer Vision')
        self.right_layout.addWidget(self.tab_widget)

        # Auto Mode Tab
        self.task_selection = QtWidgets.QWidget()  # task_selection
        self.btn_start_tasks = QtWidgets.QPushButton('start tasks', self.task_selection)  # btn_start_tasks
        self.task_layout.addWidget(self.btn_start_tasks)
        self.btn_stop_tasks = QtWidgets.QPushButton('stop tasks', self.task_selection)  # btn_stop_tasks
        self.task_layout.addWidget(self.btn_stop_tasks)
        self.btn_refresh = QtWidgets.QPushButton('refresh', self.task_selection)  # btn_refresh
        self.task_layout.addWidget(self.btn_refresh)

        self.task_button_list = []
        self.generate_task_buttons()  # Dynamically generate task buttons from config.ini

        # Manual Mode Tab
        self.btn_brake = QtWidgets.QPushButton('Brake', self.manual_controls)  # btn_brake
        self.btn_brake.setMaximumSize(QtCore.QSize(100, 100))
        self.controls_layout.addWidget(self.btn_brake, 1, 0, 1, 1)
        self.power_label = QtWidgets.QLabel('Power', self.manual_controls)  # spn_power
        self.power_label.setMaximumSize(QtCore.QSize(100, 20))
        self.power_label.setAlignment(QtCore.Qt.AlignCenter)
        self.spn_power = QtWidgets.QSpinBox(self.manual_controls)
        self.spn_power.setMaximumSize(QtCore.QSize(100, 80))
        self.spn_power.setAlignment(QtCore.Qt.AlignCenter)
        self.spn_power.setMaximum(200)
        self.spn_power.setSingleStep(20)
        self.spn_power.setProperty('value', 120)

        self.power_layout.addWidget(self.power_label)
        self.power_layout.addWidget(self.spn_power)
        self.controls_layout.addLayout(self.power_layout, 1, 2, 1, 1)
        self.rotation_label = QtWidgets.QLabel('Rotation', self.manual_controls)  # spn_rotation
        self.rotation_label.setMaximumSize(QtCore.QSize(100, 20))
        self.rotation_label.setAlignment(QtCore.Qt.AlignCenter)
        self.spn_rotation = QtWidgets.QSpinBox(self.manual_controls)
        self.spn_rotation.setMaximumSize(QtCore.QSize(100, 80))
        self.spn_rotation.setAlignment(QtCore.Qt.AlignCenter)
        self.spn_rotation.setMaximum(180)
        self.spn_rotation.setSingleStep(10)
        self.spn_rotation.setProperty('value', 20)

        self.rotation_layout.addWidget(self.rotation_label)
        self.rotation_layout.addWidget(self.spn_rotation)
        self.controls_layout.addLayout(self.rotation_layout, 1, 3, 1, 1)
        self.depth_label = QtWidgets.QLabel('Depth', self.manual_controls)  # spn_depth
        self.depth_label.setMaximumSize(QtCore.QSize(100, 20))
        self.depth_label.setAlignment(QtCore.Qt.AlignCenter)
        self.spn_depth = QtWidgets.QSpinBox(self.manual_controls)
        self.spn_depth.setMaximumSize(QtCore.QSize(100, 80))
        self.spn_depth.setAlignment(QtCore.Qt.AlignCenter)
        self.spn_depth.setMinimum(-500)
        self.spn_depth.setMaximum(500)
        self.spn_depth.setSingleStep(20)

        self.depth_layout.addWidget(self.depth_label)
        self.depth_layout.addWidget(self.spn_depth)
        self.controls_layout.addLayout(self.depth_layout, 1, 4, 1, 1)
        self.btn_strafe_l = QtWidgets.QPushButton('Strafe L', self.manual_controls)  # btn_strafe_l
        self.btn_strafe_l.setMaximumSize(QtCore.QSize(100, 100))
        self.controls_layout.addWidget(self.btn_strafe_l, 3, 0, 1, 1)
        self.btn_forward = QtWidgets.QPushButton('Forward', self.manual_controls)  # btn_forward
        self.btn_forward.setMaximumSize(QtCore.QSize(100, 100))
        self.controls_layout.addWidget(self.btn_forward, 3, 2, 1, 1)
        self.btn_strafe_r = QtWidgets.QPushButton('Strafe R', self.manual_controls)  # btn_strafe_r
        self.btn_strafe_r.setMaximumSize(QtCore.QSize(100, 100))
        self.controls_layout.addWidget(self.btn_strafe_r, 3, 3, 1, 1)
        self.btn_up = QtWidgets.QPushButton('Up', self.manual_controls)  # btn_up
        self.btn_up.setMaximumSize(QtCore.QSize(100, 100))
        self.controls_layout.addWidget(self.btn_up, 3, 4, 1, 1)
        self.btn_rotate_l = QtWidgets.QPushButton('Rotate L', self.manual_controls)  # btn_rotate_l
        self.btn_rotate_l.setMaximumSize(QtCore.QSize(100, 100))
        self.controls_layout.addWidget(self.btn_rotate_l, 4, 0, 1, 1)
        self.btn_backward = QtWidgets.QPushButton('Backward', self.manual_controls)  # btn_backward
        self.btn_backward.setMaximumSize(QtCore.QSize(100, 100))
        self.controls_layout.addWidget(self.btn_backward, 4, 2, 1, 1)
        self.btn_rotate_r = QtWidgets.QPushButton('Rotate R', self.manual_controls)  # btn_rotate_r
        self.btn_rotate_r.setMaximumSize(QtCore.QSize(100, 100))
        self.controls_layout.addWidget(self.btn_rotate_r, 4, 3, 1, 1)
        self.btn_down = QtWidgets.QPushButton('Down', self.manual_controls)  # btn_down
        self.btn_down.setMaximumSize(QtCore.QSize(100, 100))
        self.controls_layout.addWidget(self.btn_down, 4, 4, 1, 1)

        # Computer Vision Tab
        # TODO add rgb hsv sliders

        self.main_layout.addLayout(self.right_layout)

        self.right_connections()  # Controller connctions

    def left_connections(self):
        """ Controller Connections for left section"""

        # TODO
        pass

    def right_connections(self):
        """ Controller Connections for right section"""

        # Mode Selection
        self.btn_load_default.clicked.connect(self.controller.load_default_params)
        self.btn_change_params.clicked.connect(self.controller.change_params)
        self.chk_autostart.stateChanged.connect(self.checkbox_state_changed)
        self.tab_state_changed()
        self.tab_widget.currentChanged.connect(self.tab_state_changed)

        # Auto Mode Buttons
        # TODO start and stop tasks button connection
        self.btn_refresh.clicked.connect(self.refresh)

        # Manual Mode Buttons
        self.btn_forward.clicked.connect(lambda: self.controller.manual_move('forward', self.spn_power.value(), self.spn_rotation.value(), self.spn_depth.value()))
        self.btn_backward.clicked.connect(lambda: self.controller.manual_move('backward', self.spn_power.value(), self.spn_rotation.value(), self.spn_depth.value()))
        self.btn_strafe_l.clicked.connect(lambda: self.controller.manual_move('strafe_l', self.spn_power.value(), self.spn_rotation.value(), self.spn_depth.value()))
        self.btn_strafe_r.clicked.connect(lambda: self.controller.manual_move('strafe_r', self.spn_power.value(), self.spn_rotation.value(), self.spn_depth.value()))
        self.btn_rotate_l.clicked.connect(lambda: self.controller.manual_move('rotate_l', self.spn_power.value(), self.spn_rotation.value(), self.spn_depth.value()))
        self.btn_rotate_r.clicked.connect(lambda: self.controller.manual_move('rotate_r', self.spn_power.value(), self.spn_rotation.value(), self.spn_depth.value()))
        self.btn_up.clicked.connect(lambda: self.controller.manual_move('up', self.spn_power.value(), self.spn_rotation.value(), self.spn_depth.value()))
        self.btn_down.clicked.connect(lambda: self.controller.manual_move('down', self.spn_power.value(), self.spn_rotation.value(), self.spn_depth.value()))
        self.btn_brake.clicked.connect(lambda: self.controller.manual_move('brake', self.spn_power.value(), self.spn_rotation.value(), self.spn_depth.value()))

    def checkbox_state_changed(self):
        """ Triggers when checkbox is toggled"""

        if self.chk_autostart.isChecked():
            self.controller.set_auto_mode_state(1)
        else:
            self.controller.set_auto_mode_state(0)

    def tab_state_changed(self):
        """ Triggers when different tab is clicked"""

        if self.tab_widget.currentIndex() == 1:
            self.controller.manual_mode()

    def generate_task_buttons(self):
        """ Dynamically create task buttons based off of config.ini"""

        for task in self.controller.get_task_list():
            self.btn_task = QtWidgets.QPushButton(task, self.task_selection)
            self.task_layout.addWidget(self.btn_task)
            self.btn_task.clicked.connect(partial(self.controller.read_task_button, self.btn_task.text()))
            self.task_button_list.append(self.btn_task)

    def refresh(self):
        """ Trigger when refresh button is pressed"""

        for button in self.task_button_list:
            self.task_layout.removeWidget(button)
            button.deleteLater()
            button = None
        self.task_button_list = []
        self.generate_task_buttons()


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

    app = QApplication(sys.argv)
    ui = App()

    if(roscore):
        subprocess.Popen.kill(roscore)
        os.system('killall -9 rosmaster')
        os.system('killall -9 rosout')

    sys.exit(app.exec_())
