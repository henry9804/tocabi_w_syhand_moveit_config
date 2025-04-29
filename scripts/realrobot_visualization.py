#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from moveit_msgs.msg import DisplayRobotState
from geometry_msgs.msg import PolygonStamped, Transform
from sensor_msgs.msg import JointState
import message_filters
from scipy.spatial.transform import Rotation

hand_name   = ['aa2',   'mcp2',     'pip2',     'dip2',     'act2',
               'aa1',   'mcp1',     'pip1',     'dip1',     'act1',
               'aa3',   'mcp3',     'pip3',     'dip3',     'act3',
               'aa4',   'mcp4',     'pip4',     'dip4',     'act4']
hand_open   = [0.0,     0.068718,   0.008804,   0.151734,   0.054974,
               0.8,     0.068718,   0.008804,   0.151734,   0.054974,
               0.0,     0.068718,   0.008804,   0.151734,   0.054974,
               0.0,     0.068718,   0.008804,   0.151734,   0.054974]
hand_close  = [0.0,     0.830621,   0.957324,   0.837867,   0.664497,
               0.8,     0.830621,   0.957324,   0.837867,   0.664497,
               0.0,     0.830621,   0.957324,   0.837867,   0.664497,
               0.0,     0.830621,   0.957324,   0.837867,   0.664497]

class TocabiStateVisualizer:
    def __init__(self):
        self.robot_state_pub = rospy.Publisher("/current_state", DisplayRobotState, queue_size=1)
        self.target_state_pub = rospy.Publisher("/target_state", DisplayRobotState, queue_size=1)

        self.robot_state_msg = DisplayRobotState()
        self.robot_state = self.robot_state_msg.state
        self.robot_state.joint_state.header.frame_id = 'world'
        self.robot_state.multi_dof_joint_state.header.frame_id = 'world'
        self.robot_state.multi_dof_joint_state.joint_names.append('world_vjoint')
        self.robot_state.multi_dof_joint_state.transforms.append(Transform())

        self.hand_state = False  # False = open, True = close

    def robot_state_callback(self, point_msg, joint_msg):
        pelvis_pos = point_msg.polygon.points[3]
        pelvis_rpy = point_msg.polygon.points[4]
        pelvis_quat = Rotation.from_euler('xyz', [pelvis_rpy.x, pelvis_rpy.y, pelvis_rpy.z]).as_quat()

        q_body = joint_msg.position
        if self.hand_state:
            q_hand = hand_close
        else:
            q_hand = hand_open
        
        self.robot_state.joint_state.header.stamp = rospy.Time.now()
        self.robot_state.joint_state.name = [*joint_msg.name, *hand_name]
        self.robot_state.joint_state.position = [*q_body, *q_hand]
        self.robot_state.multi_dof_joint_state.transforms[0].translation = pelvis_pos
        self.robot_state.multi_dof_joint_state.transforms[0].rotation.x = pelvis_quat[0]
        self.robot_state.multi_dof_joint_state.transforms[0].rotation.y = pelvis_quat[1]
        self.robot_state.multi_dof_joint_state.transforms[0].rotation.z = pelvis_quat[2]
        self.robot_state.multi_dof_joint_state.transforms[0].rotation.w = pelvis_quat[3]

        self.robot_state_pub.publish(self.robot_state_msg)

    def joint_action_callback(self, joint_msg):
        if self.hand_state:
            q_hand = hand_close
        else:
            q_hand = hand_open
            
        target_state_msg = DisplayRobotState()
        target_state = target_state_msg.state
        target_state.joint_state.header.frame_id = 'world'
        target_state.joint_state.header.stamp = rospy.Time.now()
        target_state.multi_dof_joint_state = self.robot_state.multi_dof_joint_state
        target_state.joint_state.name = [*joint_msg.name, *hand_name]
        target_state.joint_state.position = [*joint_msg.position, *q_hand]

        self.target_state_pub.publish(target_state_msg)

    def hand_action_callback(self, hand_msg):
        self.hand_state = hand_msg.data

if __name__ == '__main__':
    rospy.init_node('tocabi_visualize', anonymous=True)

    visualizer = TocabiStateVisualizer()

    point_sub = message_filters.Subscriber("/tocabi/point", PolygonStamped)
    joint_sub = message_filters.Subscriber("/tocabi/jointstates", JointState)
    ts = message_filters.ApproximateTimeSynchronizer([point_sub, joint_sub], 10, 0.01)
    ts.registerCallback(visualizer.robot_state_callback)

    joint_action_sub = rospy.Subscriber("/tocabi/act/joint_target", JointState, visualizer.joint_action_callback)
    hand_action_sub = rospy.Subscriber("/tocabi_hand/on", Bool, visualizer.hand_action_callback)

    rospy.spin()
