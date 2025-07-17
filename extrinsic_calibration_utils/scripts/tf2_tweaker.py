#! /usr/bin/env python3
import os
import argparse
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import sys
import time
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from colorama import Fore, Style
from enum import Enum
import numpy as np
from scipy.spatial.transform import Rotation


def check_xyzypr_input(xyzypr_input, desired_len=None):
    try:
        input_list = xyzypr_input.split(', ')
        if not desired_len:
            desired_len = len(input_list)
        if len(input_list) != desired_len:
            return False
        for i in range(desired_len):
            float(input_list[i])
        return True
    except ValueError:
        return False
    

def xyzypr_string_to_vals(xyzypr_input):
    if not check_xyzypr_input(xyzypr_input):
        return None
    input_list = xyzypr_input.split(', ')
    if len(input_list) == 3:
        return [float(input_list[0]), float(input_list[1]), float(input_list[2])]
    else:
        xyz = [float(input_list[0]), float(input_list[1]), float(input_list[2])]
        ypr = [float(input_list[3]), float(input_list[4]), float(input_list[5])]
        return xyz, ypr


class tf_edit_state(Enum):
    both = 0
    xyz = 1
    ypr = 2


class tf_tweaker_tool(Node):
    def __init__(self):
        super().__init__('tf_tweaker')
        self.logger = self.get_logger()

        # Initialize the transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.base_tf_frame = ''
        self.child_tf_frame = ''
        self.xyz = [0, 0, 0]
        self.ypr = [0, 0, 0]
        self.edit_state = tf_edit_state.both
        self.desired_xyzypr_len = 6
        self.input_msg_both = f'Please enter TF transform in the format x, y, z, yaw, pitch, roll. xyz in meters, angles in degrees: '
        self.input_msg_xyz = f'Please enter position in the format x, y, z (in meters): '
        self.input_msg_ypr = f'Please enter orientation in the format yaw, pitch, roll (in degrees): '
        self.input_msg = self.input_msg_both
        self.wrong_msg = f'Unreadable input'

    def update_edit_state(self, target_state):
        if target_state == tf_edit_state.both:
            self.desired_xyzypr_len = 6
            self.edit_state = target_state
            self.input_msg = self.input_msg_both
        elif target_state == tf_edit_state.xyz or target_state == tf_edit_state.ypr:
            self.desired_xyzypr_len = 3
            self.edit_state = target_state
            if target_state == tf_edit_state.xyz:
                self.input_msg = self.input_msg_xyz
            else:
                self.input_msg = self.input_msg_ypr
        else:
            print(f'could not update tf_edit_state to {target_state}')
            return


    def get_user_input(self, check_func):
        while True:
            print(Style.RESET_ALL)
            print("Press 'q' to quit, 'p' to edit position, 'a' to edit orientation, 'b' to edit both orientation and position")
            resp = input(self.input_msg)
            if resp.lower() == 'q':
                exit()
            elif resp.lower() == 'p':
                self.update_edit_state(tf_edit_state.xyz)
            elif resp.lower() == 'a':
                self.update_edit_state(tf_edit_state.ypr)
            elif resp.lower() == 'b':
                self.update_edit_state(tf_edit_state.both)
            elif check_func(resp, self.desired_xyzypr_len):
                return resp
            else:
                print(Fore.RED + self.wrong_msg)

    def broadcast_tf(self):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_tf_frame
        t.child_frame_id = self.child_tf_frame

        t.transform.translation.x = self.xyz[0]
        t.transform.translation.y = self.xyz[1]
        t.transform.translation.z = self.xyz[2]

        rot = Rotation.from_euler('zyx', self.ypr, degrees=True)
        print(rot.as_matrix())
        q = rot.as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def main_terminal(self):

        # Get/start with tf frames to use
        # get transform from user
        # publish transform
        # loop:
            # (keep publishing? or is once fine?)
            # get new transform from the user
            # update transform
        while True:
            xyzypr_string = self.get_user_input(check_xyzypr_input)
            if self.edit_state == tf_edit_state.both:
                self.xyz, self.ypr = xyzypr_string_to_vals(xyzypr_string)
            elif self.edit_state == tf_edit_state.xyz:
                self.xyz = xyzypr_string_to_vals(xyzypr_string)
            else:
                self.ypr = xyzypr_string_to_vals(xyzypr_string)
            self.broadcast_tf()
            print(Fore.GREEN + f'TF successfully broadcasted: position {self.xyz}, orientation {self.ypr}')


def main():
    parser = argparse.ArgumentParser(
        description='Run tf2_tweaker with a base and child tf frame.'
    )
    parser.add_argument('base_tf', help='The base TF frame')
    parser.add_argument('child_tf', help='The child TF frame')
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    tf_tweak = tf_tweaker_tool()
    tf_tweak.base_tf_frame = args.base_tf
    tf_tweak.child_tf_frame = args.child_tf
    tf_tweak.main_terminal()
    rclpy.spin(tf_tweak)


if __name__ == '__main__':
    main()

# 40, 0, 0, 3, 20, 180
# TF successfully broadcasted: position [28.55, -26.1, 0.4], orientation [2.65, 21.0, 190.45]