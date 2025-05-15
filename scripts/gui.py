#!/usr/bin/env python

import sys, time
import numpy as np
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from tocabi_msgs.msg import positionCommand
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QHBoxLayout, QPushButton, QWidget, QLabel
from functools import partial
from utils import JOINT_NAMES, INIT_POSE, READY_POSE, cubic

class ROSPublisherApp(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

        self.joint_subscriber = rospy.Subscriber('/tocabi/jointstates', JointState, callback=self.joint_callback)
        self.current_joint_pos = None
        self.current_joint_vel = None

        self.joint_target_pub = rospy.Publisher("/tocabi/act/joint_target", JointState, queue_size=10)
        self.position_command_pub = rospy.Publisher("/tocabi/positioncommand", positionCommand, queue_size=10)
        self.hand_publisher = rospy.Publisher('/tocabi_hand/on', Bool, queue_size=10)
        self.act_publisher = rospy.Publisher('/tocabi/act/terminate', Bool, queue_size=10)

        rospy.init_node('pyqt_ros_publisher', anonymous=True)

    def joint_callback(self, msg: JointState):
        self.current_joint_pos = msg.position
        self.current_joint_vel = msg.velocity

    def init_ui(self):
        self.setWindowTitle("ROS Publisher GUI")
        self.setGeometry(1700, 100, 200, 400)

        layout = QVBoxLayout()
        layout.addStretch(1)

        # TOCABI
        tocabi_layout = QVBoxLayout()
        tocabi_label = QLabel("TOCABI")
        tocabi_label.setStyleSheet("font-size: 16px; font-weight: bold; text-align: center;")
        tocabi_label.setAlignment(Qt.AlignCenter)
        tocabi_layout.addWidget(tocabi_label)

        tocabi_layout.addSpacing(10)

        init_pose_button = QPushButton("init pose")
        init_pose_button.clicked.connect(partial(self.tocabi_joint_traj, INIT_POSE, 4.0))
        tocabi_layout.addWidget(init_pose_button)

        ready_pose_button = QPushButton("ready pose")
        ready_pose_button.clicked.connect(partial(self.tocabi_joint_traj, READY_POSE, 4.0))
        tocabi_layout.addWidget(ready_pose_button)

        layout.addLayout(tocabi_layout)
        layout.addStretch(1)

        # Hand
        hand_layout = QVBoxLayout()
        hand_label = QLabel("Hand")
        hand_label.setStyleSheet("font-size: 16px; font-weight: bold; text-align: center;")
        hand_label.setAlignment(Qt.AlignCenter)
        hand_layout.addWidget(hand_label)

        hand_layout.addSpacing(10)

        grasp_button = QPushButton("Grasp")
        grasp_button.clicked.connect(partial(self.publish_hand_msg, True))
        hand_layout.addWidget(grasp_button)

        hand_layout.addSpacing(10)

        stretch_button = QPushButton("Stretch")
        stretch_button.clicked.connect(partial(self.publish_hand_msg, False))
        hand_layout.addWidget(stretch_button)

        layout.addLayout(hand_layout)
        layout.addStretch(1)

        # ACT
        act_layout = QVBoxLayout()
        act_label = QLabel("ACT")
        act_label.setStyleSheet("font-size: 16px; font-weight: bold; text-align: center;")
        act_label.setAlignment(Qt.AlignCenter)
        act_layout.addWidget(act_label)

        act_layout.addSpacing(10)

        start_button = QPushButton("Start")
        start_button.clicked.connect(partial(self.publish_act_msg, False))
        act_layout.addWidget(start_button)

        act_layout.addSpacing(10)

        end_button = QPushButton("End")
        end_button.clicked.connect(partial(self.publish_act_msg, True))
        act_layout.addWidget(end_button)

        layout.addLayout(act_layout)
        layout.addStretch(1)

        self.setLayout(layout)

    def tocabi_joint_traj(self, target, duration):
        # ver 1. publish JointState message
        start = time.time()
        init_pos = np.array(self.current_joint_pos)
        init_vel = np.array(self.current_joint_vel)

        target_joint_msg = JointState()
        target_joint_msg.name = JOINT_NAMES
        while(time.time() - start < duration + 0.2):
            target_pos, target_vel = cubic(time.time(), start, start + duration, init_pos, target, init_vel, np.zeros_like(init_vel))
            target_joint_msg.position = target_pos
            target_joint_msg.velocity = target_vel
            self.joint_target_pub.publish(target_joint_msg)
            rospy.sleep(0.1)

        # ver 2. publish positionCommand message
        # position_command_msg = positionCommand()
        # position_command_msg.position = target
        # position_command_msg.traj_time = duration
        # position_command_msg.gravity = True
        # position_command_msg.relative = False
        # self.position_command_pub.publish(position_command_msg)

    def publish_hand_msg(self, data):
        msg = Bool()
        msg.data = data
        self.hand_publisher.publish(msg)
        print(f"Hand On: {msg.data}")

    def publish_act_msg(self, data):
        msg = Bool()
        msg.data = data
        self.act_publisher.publish(msg)
        print(f"ACT Terminate: {msg.data}")

if __name__ == "__main__":
    app = QApplication(sys.argv)

    try:
        gui = ROSPublisherApp()
        gui.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
