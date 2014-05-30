from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame, QGroupBox, QInputDialog, QMessageBox
from python_qt_binding.QtCore import QObject, QSignalMapper, qWarning, Signal

class CreateMovement:
    _widget = QWidget(None)
    _widget.setWindowTitle('Create an arm movement')
    _widget.setObjectName('ArmMoveGUI')
    
    def __init__(self, SimpleGUI):
        # Main layout element for GUI
        large_box = QtGui.QVBoxLayout()
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.blue)
        
        # Create Button box for GUI
        # addItem(item, int row, int column, int rowSpan = 1, int columnSpan = 1, Qt::Alignment alignment = 0)
	primary_box = QtGui.QGridLayout()
        # Add The Widgets to the grid
        self.name_label = QtGui.QLabel('Movement Name: ')
        self.name_label.setPalette(palette)
        primary_box.addWidget(self.name_label, 1, 1, 1, 2)
        self.name_in = QtGui.QLineEdit('Default')
        primary_box.addWidget(self.name_in, 1, 3, 1, 8)

        self.name_label = QtGui.QLabel('Left Arm Positions')
        self.name_label.setPalette(palette)
        primary_box.addWidget(self.name_label, 2, 1, 1, 3)

        self.name_label = QtGui.QLabel('Start Time')
        self.name_label.setPalette(palette)
        primary_box.addWidget(self.name_label, 2, 4, 1, 1)

        self.name_label = QtGui.QLabel('Right Arm Positions')
        self.name_label.setPalette(palette)
        primary_box.addWidget(self.name_label, 2, 6, 1, 3)

        self.name_label = QtGui.QLabel('Start Time')
        self.name_label.setPalette(palette)
        primary_box.addWidget(self.name_label, 2, 9, 1, 1)

        self.l_arm_pose_box = QtGui.QComboBox()
        for i in range(0, SimpleGUI.l_arm_pose_box.count()):
            self.l_arm_pose_box.addItem(SimpleGUI.l_arm_pose_box.itemText(i))
        primary_box.addWidget(self.l_arm_pose_box, 3, 1, 1, 3)

        self.l_time_in = QtGui.QLineEdit('1.0')
        primary_box.addWidget(self.l_time_in, 3, 4, 1, 1)

        self.l_add_btn = QtGui.QPushButton('Add')
        self.l_add_btn.clicked.connect(self.add_l_pose)      
	primary_box.addWidget(self.l_add_btn, 3, 5, 1, 1)

        self.r_arm_pose_box = QtGui.QComboBox()
        for i in range(0, SimpleGUI.r_arm_pose_box.count()):
            self.r_arm_pose_box.addItem(SimpleGUI.r_arm_pose_box.itemText(i))
        primary_box.addWidget(self.r_arm_pose_box, 3, 6, 1, 3)

        self.r_time_in = QtGui.QLineEdit('1.0')
        primary_box.addWidget(self.r_time_in, 3, 9, 1, 1)

        primary_box.addWidget(QtGui.QPushButton('Add'), 3, 10, 1, 1)

        self.l_text_box = QtGui.QPlainTextEdit()
        self.l_text_box.setReadOnly(True)
        primary_box.addWidget(self.l_text_box, 4, 1, 1, 5)

        self.r_text_box = QtGui.QPlainTextEdit()
        self.r_text_box.setReadOnly(True)
        primary_box.addWidget(self.r_text_box, 4, 6, 1, 5)

        primary_box.addWidget(QtGui.QPushButton('Save'), 5, 5, 1, 1)
        primary_box.addWidget(QtGui.QPushButton('Exit'), 5, 6, 1, 1)
        large_box.addLayout(primary_box)
        large_box.addItem(QtGui.QSpacerItem(100,20))
        
        # Puts layout in widget and adds widget name for RosGUI to identify
        self._widget.setLayout(large_box)
    
    # Will create a button, with all buttons using same event trigger
    def create_button(self, name):
        btn = QtGui.QPushButton(name, self._widget)
        #btn.clicked.connect(self.command_cb)
        return btn

    def add_l_pose(self):
        self.l_text_box.insertPlainText('Test')
        self.l_text_box.repaint()

            
