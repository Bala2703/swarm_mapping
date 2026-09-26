import sys
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QComboBox, QCheckBox, QPushButton, 
                             QDoubleSpinBox, QGroupBox, QFormLayout)
from PyQt5.QtCore import Qt, QTimer

class SwarmTeleopNode(Node):
    def __init__(self):
        super().__init__('swarm_teleop_ui_node')
        
        self.robots = ['mapper_1', 'scout_1', 'scout_2']
        self.active_robot = 'mapper_1'
        self.broadcast_all = False
        
        self.pubs = {}
        for r in self.robots:
            self.pubs[r] = self.create_publisher(Twist, f'/{r}/cmd_vel', 10)

    def publish_command(self, x, z, linear_speed, angular_speed):
        msg = Twist()
        msg.linear.x = float(x * linear_speed)
        msg.angular.z = float(z * angular_speed)

        if self.broadcast_all:
            for pub in self.pubs.values():
                pub.publish(msg)
        else:
            self.pubs[self.active_robot].publish(msg)


class DrivePad(QLabel):
    """ Custom focusable pad to catch keyboard events without widget interference. """
    def __init__(self, parent_ui):
        super().__init__()
        self.ui = parent_ui
        self.setAlignment(Qt.AlignCenter)
        self.setFocusPolicy(Qt.StrongFocus)
        self.set_unfocused_style()

    def set_unfocused_style(self):
        self.setStyleSheet("""
            background-color: #2c3e50; 
            color: #ecf0f1; 
            font-size: 16px; 
            border: 2px solid #34495e;
        """)
        self.setText("DRIVE PAD\n\n[ CLICK HERE TO DRIVE ]\n\nRobot will stop if you click away.")

    def set_focused_style(self):
        self.setStyleSheet("""
            background-color: #27ae60; 
            color: white; 
            font-weight: bold; 
            font-size: 16px; 
            border: 4px solid #2ecc71;
        """)
        self.update_text()

    def update_text(self):
        if self.hasFocus():
            self.setText(f"ACTIVE\n\nDir X: {self.ui.dir_x} | Dir Z: {self.ui.dir_z}\nPress 'k' to stop.")

    def focusInEvent(self, event):
        self.set_focused_style()

    def focusOutEvent(self, event):
        # Safety feature: Stop the robot if the window loses focus
        self.ui.dir_x = 0.0
        self.ui.dir_z = 0.0
        self.set_unfocused_style()

    def keyPressEvent(self, event):
        self.ui.handle_keypress(event)
        self.update_text()


class SwarmUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Swarm Keyboard Teleop")
        self.resize(400, 500)
        
        self.ros_node = SwarmTeleopNode()

        # Movement State
        self.dir_x = 0.0
        self.dir_z = 0.0
        self.lin_speed = 0.5
        self.ang_speed = 1.0

        # Standard teleop_twist_keyboard mappings
        self.move_bindings = {
            Qt.Key_I: (1.0, 0.0),
            Qt.Key_O: (1.0, -1.0),
            Qt.Key_J: (0.0, 1.0),
            Qt.Key_L: (0.0, -1.0),
            Qt.Key_U: (1.0, 1.0),
            Qt.Key_Comma: (-1.0, 0.0),
            Qt.Key_Period: (-1.0, -1.0),
            Qt.Key_M: (-1.0, 1.0),
        }

        # Speed scaling mappings
        self.speed_bindings = {
            Qt.Key_Q: (1.1, 1.1),  # increase overall
            Qt.Key_Z: (0.9, 0.9),  # decrease overall
            Qt.Key_W: (1.1, 1.0),  # increase linear
            Qt.Key_X: (0.9, 1.0),  # decrease linear
            Qt.Key_E: (1.0, 1.1),  # increase angular
            Qt.Key_C: (1.0, 0.9),  # decrease angular
        }

        self.setup_ui()

        # 10Hz Publish Timer (Robots need constant cmd_vel to not timeout)
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_loop)
        self.timer.start(100)

    def setup_ui(self):
        layout = QVBoxLayout()

        # Target Selector
        target_group = QGroupBox("Target Selection")
        target_layout = QHBoxLayout()
        self.selector = QComboBox()
        self.selector.addItems(['mapper_1', 'scout_1', 'scout_2'])
        self.selector.currentTextChanged.connect(self.change_active_robot)
        
        self.broadcast_checkbox = QCheckBox("Drive ALL (Broadcast)")
        self.broadcast_checkbox.stateChanged.connect(self.toggle_broadcast)
        
        target_layout.addWidget(QLabel("Control:"))
        target_layout.addWidget(self.selector)
        target_layout.addWidget(self.broadcast_checkbox)
        target_group.setLayout(target_layout)
        layout.addWidget(target_group)

        # Speed Controls
        speed_group = QGroupBox("Speed Adjustments")
        speed_layout = QFormLayout()
        
        self.lin_spin = QDoubleSpinBox()
        self.lin_spin.setRange(0.0, 3.0)
        self.lin_spin.setSingleStep(0.1)
        self.lin_spin.setValue(self.lin_speed)
        self.lin_spin.valueChanged.connect(self.sync_speeds_from_ui)

        self.ang_spin = QDoubleSpinBox()
        self.ang_spin.setRange(0.0, 5.0)
        self.ang_spin.setSingleStep(0.1)
        self.ang_spin.setValue(self.ang_speed)
        self.ang_spin.valueChanged.connect(self.sync_speeds_from_ui)

        speed_layout.addRow("Linear Speed (m/s):", self.lin_spin)
        speed_layout.addRow("Angular Speed (rad/s):", self.ang_spin)
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)

        # Help Text
        help_text = QLabel(
            "<b>Movement:</b> u i o | j k l | m , . (k to stop)<br>"
            "<b>Speeds:</b> q/z (all), w/x (linear), e/c (angular)"
        )
        help_text.setAlignment(Qt.AlignCenter)
        layout.addWidget(help_text)

        # Drive Pad
        self.drive_pad = DrivePad(self)
        self.drive_pad.setMinimumHeight(150)
        layout.addWidget(self.drive_pad)

        # Emergency Stop
        self.stop_btn = QPushButton("HARD STOP")
        self.stop_btn.setStyleSheet("background-color: darkred; color: white; font-weight: bold; padding: 10px;")
        self.stop_btn.clicked.connect(self.hard_stop)
        layout.addWidget(self.stop_btn)

        self.setLayout(layout)

    def change_active_robot(self, robot_name):
        self.ros_node.active_robot = robot_name

    def toggle_broadcast(self, state):
        self.ros_node.broadcast_all = (state == Qt.Checked)

    def sync_speeds_from_ui(self):
        self.lin_speed = self.lin_spin.value()
        self.ang_speed = self.ang_spin.value()

    def sync_ui_from_speeds(self):
        # Block signals temporarily to prevent infinite loop
        self.lin_spin.blockSignals(True)
        self.ang_spin.blockSignals(True)
        self.lin_spin.setValue(self.lin_speed)
        self.ang_spin.setValue(self.ang_speed)
        self.lin_spin.blockSignals(False)
        self.ang_spin.blockSignals(False)

    def handle_keypress(self, event):
        key = event.key()
        
        if key in self.move_bindings:
            self.dir_x, self.dir_z = self.move_bindings[key]
        elif key == Qt.Key_K:
            self.dir_x = 0.0
            self.dir_z = 0.0
        elif key in self.speed_bindings:
            lin_mult, ang_mult = self.speed_bindings[key]
            self.lin_speed *= lin_mult
            self.ang_speed *= ang_mult
            self.sync_ui_from_speeds()

    def hard_stop(self):
        self.dir_x = 0.0
        self.dir_z = 0.0
        self.drive_pad.update_text()

    def publish_loop(self):
        self.ros_node.publish_command(self.dir_x, self.dir_z, self.lin_speed, self.ang_speed)

def main():
    rclpy.init()
    
    app = QApplication(sys.argv)
    ui = SwarmUI()
    ui.show()

    ros_thread = threading.Thread(target=rclpy.spin, args=(ui.ros_node,), daemon=True)
    ros_thread.start()

    exit_code = app.exec_()
    ui.ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()