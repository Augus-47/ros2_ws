#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from legend_interfaces.msg import VehicleCmd


class TwistToVehicleCmdNode(Node):
    def __init__(self):
        super().__init__('twist_to_vehiclecmd_node')

        self.declare_parameter('wheelbase', 0.8)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_steering', 0.6)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('vehicle_cmd_topic', '/vehicle/cmd')
        self.declare_parameter('zero_speed_eps', 0.01)

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.vehicle_cmd_topic = str(self.get_parameter('vehicle_cmd_topic').value)
        self.zero_speed_eps = float(self.get_parameter('zero_speed_eps').value)

        self.sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            20
        )

        self.pub = self.create_publisher(
            VehicleCmd,
            self.vehicle_cmd_topic,
            20
        )

        self.get_logger().info(
            f'Started twist_to_vehiclecmd_node | '
            f'wheelbase={self.wheelbase:.3f}, '
            f'max_speed={self.max_speed:.3f}, '
            f'max_steering={self.max_steering:.3f}, '
            f'cmd_vel_topic={self.cmd_vel_topic}, '
            f'vehicle_cmd_topic={self.vehicle_cmd_topic}'
        )

    def clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def cmd_callback(self, msg: Twist):
        v = float(msg.linear.x)
        wz = float(msg.angular.z)

        v = self.clamp(v, -self.max_speed, self.max_speed)

        if abs(v) < self.zero_speed_eps:
            steering = 0.0
        else:
            steering = math.atan(self.wheelbase * wz / v)

        steering = self.clamp(steering, -self.max_steering, self.max_steering)

        out = VehicleCmd()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'

        out.velocity = float(v)
        out.steering = float(steering)
        out.brake = 0.0
        out.gears = 0.0
        out.led = 'no'

        self.pub.publish(out)

        self.get_logger().debug(
            f'cmd_vel -> vehicle_cmd | '
            f'vx={v:.3f}, wz={wz:.3f}, steering={steering:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TwistToVehicleCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

