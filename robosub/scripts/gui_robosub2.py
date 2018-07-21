import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QColorDialog
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QColor

# from modules.controller.gui_controller import Controller


class App(QWidget):

    def __init__(self):
        super(App, self).__init__()
        # self.Controller = Controller()  # Initialize gui controller
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

        # Layouts
        left_layout = QtWidgets.QVBoxLayout()
        display_layout = QtWidgets.QHBoxLayout()
        camera_layout = QtWidgets.QHBoxLayout()

        # Widgets
        display0 = QtWidgets.QGraphicsView(self)  # display0
        display_layout.addWidget(display0)
        display1 = QtWidgets.QGraphicsView(self)  # display1
        display_layout.addWidget(display1)

        left_layout.addLayout(display_layout)  # end of display_layout

        btn_camera0 = QtWidgets.QPushButton('Camera 0', self)  # btn_camera0
        camera_layout.addWidget(btn_camera0)
        btn_camera1 = QtWidgets.QPushButton('Camera 1', self)  # btn_camera1
        camera_layout.addWidget(btn_camera1)
        btn_camera2 = QtWidgets.QPushButton('Camera 2', self)  # btn_camera2
        camera_layout.addWidget(btn_camera2)

        left_layout.addLayout(camera_layout)  # end of camera_layout

        sensor_data = QtWidgets.QTableView(self)  # sensor_data
        sensor_data.setShowGrid(True)
        # sensor_data.setRowCount(4)
        # sensor_data.setColumnCount(7)
        sensor_data.horizontalHeader().setVisible(False)
        sensor_data.horizontalHeader().setHighlightSections(False)
        sensor_data.verticalHeader().setVisible(False)
        sensor_data.verticalHeader().setHighlightSections(False)
        left_layout.addWidget(sensor_data)
        messages = QtWidgets.QTextBrowser(self)  # messages
        left_layout.addWidget(messages)

        self.main_layout.addLayout(left_layout)  # end of left_layout

    def right_section(self):

        # Layouts
        right_layout = QtWidgets.QVBoxLayout()
        mode_layout = QtWidgets.QHBoxLayout()

            # Auto Mode Tab
        task_selection = QtWidgets.QWidget()  # task_selection
        task_layout = QtWidgets.QVBoxLayout(task_selection)

            # Manual Mode Tab
        manual_controls = QtWidgets.QWidget()  # manual_controls
        controls_layout = QtWidgets.QGridLayout(manual_controls)
        controls_layout.setContentsMargins(0, 0, 0, 0)
        controls_layout.setHorizontalSpacing(4)
        controls_layout.setVerticalSpacing(3)
        # power_layout = QtWidgets.QVBoxLayout()
        # rotation_layout = QtWidgets.QVBoxLayout()
        # depth_layout = QtWidgets.QVBoxLayout()

            # Computer Vision Tab
            # TODO

        # Widgets
        btn_load_default = QtWidgets.QPushButton('Load Default Params', self)  # btn_load_default
        mode_layout.addWidget(btn_load_default)
        btn_change_params = QtWidgets.QPushButton('Change Params', self)  # btn_change_params
        mode_layout.addWidget(btn_change_params)
        chk_autostart = QtWidgets.QCheckBox('Start in Auto Mode', self)  # chk_autostart
        mode_layout.addWidget(chk_autostart)

        right_layout.addLayout(mode_layout)  # end of mode layout

        tab_widget = QtWidgets.QTabWidget(self)  # tab_widget
        tab_widget.addTab(task_selection, 'Auto Mode')
        tab_widget.addTab(manual_controls, 'Manual Mode')
        right_layout.addWidget(tab_widget)

            # Auto Mode Tab
        # task_selection = QtWidgets.QWidget()  # task_selection
        btn_start_tasks = QtWidgets.QPushButton('start tasks', task_selection)  # btn_start_tasks
        task_layout.addWidget(btn_start_tasks)
        btn_stop_tasks = QtWidgets.QPushButton('stop tasks', task_selection)  # btn_stop_tasks
        task_layout.addWidget(btn_stop_tasks)
        # TODO dynamically add buttons

            # Manual Mode Tab
        btn_forward = QtWidgets.QPushButton('forward', manual_controls)  # btn_forward
        btn_forward.setMaximumSize(QtCore.QSize(100, 100))
        controls_layout.addWidget(btn_forward, 3, 2, 1, 1)
        # TODO buttons

            # Computer Vision Tab
            # TODO

        self.main_layout.addLayout(right_layout)  # end of right_layout


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ui = App()
    sys.exit(app.exec_())
