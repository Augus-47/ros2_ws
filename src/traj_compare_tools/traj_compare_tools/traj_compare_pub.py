#!/usr/bin/env python3
import csv

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


def load_csv(csv_path):
    pts = []
    frame_id = 'map'

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame_id = row['frame_id']
            pts.append((float(row['x']), float(row['y'])))

    return frame_id, pts


class TrajComparePublisher(Node):
    def __init__(self):
        super().__init__('traj_compare_publisher')

        self.declare_parameter('csv_bev_off', '/tmp/traj_bev_off.csv')
        self.declare_parameter('csv_bev_on', '/tmp/traj_bev_on.csv')
        self.declare_parameter('default_frame_id', 'map')

        self.csv_bev_off = self.get_parameter('csv_bev_off').value
        self.csv_bev_on = self.get_parameter('csv_bev_on').value
        self.default_frame_id = self.get_parameter('default_frame_id').value

        self.frame_off, self.pts_off = load_csv(self.csv_bev_off)
        self.frame_on, self.pts_on = load_csv(self.csv_bev_on)

        self.path_off_pub = self.create_publisher(Path, '/traj_bev_off', 1)
        self.path_on_pub = self.create_publisher(Path, '/traj_bev_on', 1)
        self.marker_off_pub = self.create_publisher(Marker, '/traj_bev_off_marker', 1)
        self.marker_on_pub = self.create_publisher(Marker, '/traj_bev_on_marker', 1)

        self.timer = self.create_timer(0.5, self.publish_all)

        self.get_logger().info(
            f'Loaded OFF={len(self.pts_off)} points from {self.csv_bev_off}, '
            f'ON={len(self.pts_on)} points from {self.csv_bev_on}'
        )

    def make_path(self, pts, frame_id):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        for x, y in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        return msg

    def make_marker(self, pts, frame_id, ns, marker_id, r, g, b):
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.ns = ns
        msg.id = marker_id
        msg.type = Marker.LINE_STRIP
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.03
        msg.color.a = 1.0
        msg.color.r = r
        msg.color.g = g
        msg.color.b = b

        for x, y in pts:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.02
            msg.points.append(p)

        return msg

    def publish_all(self):
        frame_off = self.frame_off if self.frame_off else self.default_frame_id
        frame_on = self.frame_on if self.frame_on else self.default_frame_id

        self.path_off_pub.publish(self.make_path(self.pts_off, frame_off))
        self.path_on_pub.publish(self.make_path(self.pts_on, frame_on))

        self.marker_off_pub.publish(
            self.make_marker(self.pts_off, frame_off, 'traj_compare', 0, 1.0, 0.0, 0.0)
        )
        self.marker_on_pub.publish(
            self.make_marker(self.pts_on, frame_on, 'traj_compare', 1, 0.0, 1.0, 0.0)
        )


def main():
    rclpy.init()
    node = TrajComparePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
