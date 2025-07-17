#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
import time
import os

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import sensor_msgs_py.point_cloud2 as pc2
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
        self.create_subscription(PointCloud2, '/lidar/points', self.cloud_callback, 10)

        self.get_logger().info("Node initialized. Press Enter to capture image and point cloud.")
        self.input_thread = threading.Thread(target=self.wait_for_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def image_callback(self, msg):
        with self.image_lock:
            self.image_msg = msg

    def cloud_callback(self, msg):
        with self.cloud_lock:
            self.cloud_msg = msg

    def wait_for_input_loop(self):
        while rclpy.ok():
            input("Press Enter to capture...")
            with self.image_lock:
                image_msg = self.image_msg
            with self.cloud_lock:
                cloud_msg = self.cloud_msg

            if image_msg is None or cloud_msg is None:
                self.get_logger().warn("No data received yet.")
                continue

            ts = image_msg.header.stamp.sec * 1_000_000_000 + image_msg.header.stamp.nanosec

            # Save image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            image_filename = f"{ts}_image.png"
            cv2.imwrite(image_filename, cv_image)

            # Save point cloud
            cloud_points = np.array([
                [x, y, z] for x, y, z, *_ in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z"))
            ], dtype=np.float32)

            cloud_filename = f"{ts}_lidar.pcd"
            with open(cloud_filename, 'w') as f:
                f.write("VERSION .7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(cloud_points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(cloud_points)}\n")
                f.write("DATA ascii\n")
                for pt in cloud_points:
                    f.write(f"{pt[0]} {pt[1]} {pt[2]}\n")

            self.get_logger().info(f"Saved: {image_filename}, {cloud_filename}")

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

