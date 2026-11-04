#!/usr/bin/env python3
import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrajRecorder(Node):
    def __init__(self):
        super().__init__('traj_recorder')

        self.declare_parameter('pose_topic', '/pcl_pose')
        self.declare_parameter('csv_path', '/tmp/traj.csv')
        self.declare_parameter('min_dist', 0.02)
        self.declare_parameter('use_odom_msg', False)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.csv_path = self.get_parameter('csv_path').value
        self.min_dist = float(self.get_parameter('min_dist').value)
        self.use_odom_msg = bool(self.get_parameter('use_odom_msg').value)

        self.rows = []
        self.last_x = None
        self.last_y = None

        if self.use_odom_msg:
            self.sub = self.create_subscription(
                Odometry, self.pose_topic, self.odom_cb, 10)
            self.get_logger().info(f'Listening Odometry topic: {self.pose_topic}')
        else:
            self.sub = self.create_subscription(
                PoseWithCovarianceStamped, self.pose_topic, self.pose_cb, 10)
            self.get_logger().info(f'Listening PoseWithCovarianceStamped topic: {self.pose_topic}')

        self.get_logger().info(f'CSV output: {self.csv_path}')

    def should_store(self, x, y):
        if self.last_x is None:
            return True
        return math.hypot(x - self.last_x, y - self.last_y) >= self.min_dist

    def store(self, stamp_sec, frame_id, x, y, yaw):
        self.rows.append([stamp_sec, frame_id, x, y, yaw])
        self.last_x = x
        self.last_y = y

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        if self.should_store(x, y):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.store(t, msg.header.frame_id, x, y, yaw)

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        if self.should_store(x, y):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.store(t, msg.header.frame_id, x, y, yaw)

    def save_csv(self):
        path = Path(self.csv_path)
        path.parent.mkdir(parents=True, exist_ok=True)

        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'frame_id', 'x', 'y', 'yaw'])
            writer.writerows(self.rows)

        self.get_logger().info(f'Saved {len(self.rows)} points to {self.csv_path}')


def main():
    rclpy.init()
    node = TrajRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
