#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import time
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class OneShotRecorder(Node):
    def __init__(self):
        super().__init__('one_shot_recorder')

        self.bridge = CvBridge()
        self.image_msg = None
        self.cloud_msg = None
        self.image_lock = threading.Lock()
        self.cloud_lock = threading.Lock()

        self.create_subscription(Image, 'image', self.image_callback, 10)

        self.get_logger().info("Node initialized. Press Enter to capture image.")
        self.input_thread = threading.Thread(target=self.wait_for_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def image_callback(self, msg):
        with self.image_lock:
            self.image_msg = msg

    def wait_for_input_loop(self):
        while rclpy.ok():
            input("Press Enter to capture...")
            with self.image_lock:
                image_msg = self.image_msg

            if image_msg is None:
                self.get_logger().warn("No data received yet.")
                continue

            ts = image_msg.header.stamp.sec * 1_000_000_000 + image_msg.header.stamp.nanosec

            # Save image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            image_filename = f"{ts}_image.png"
            cv2.imwrite(image_filename, cv_image)

            self.get_logger().info(f"Saved: {image_filename}")

def main(args=None):
    rclpy.init(args=args)
    node = OneShotRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

