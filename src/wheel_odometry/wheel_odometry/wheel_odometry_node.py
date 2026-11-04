	#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from car_msg.msg import VehicleState  # 你的自定义消息包


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        # 参数
        self.declare_parameter('wheelbase', 0.80)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', 'wheel_odom')
        self.declare_parameter('vehicle_state_topic', '/vehicle_info')

        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.vehicle_state_topic = self.get_parameter('vehicle_state_topic').get_parameter_value().string_value

        # 订阅小车状态
        self.state_sub = self.create_subscription(
            VehicleState,
            self.vehicle_state_topic,
            self.state_callback,
            10
        )

        # 发布轮式里程计（Odometry）
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # 里程计初始位姿
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 上一帧时间（使用 ROS 时钟，单位 ns）
        self.last_time_ns = None

        self.get_logger().info(
            f'Wheel odometry node started. wheelbase={self.wheelbase} m, '
            f'Odometry stamp: ROS clock now()'
        )

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        """平面 yaw -> 四元数"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def state_callback(self, msg: VehicleState):
        # 使用 ROS 时间（如果 use_sim_time=false，则是系统时间；true 则是仿真时间）
        now = self.get_clock().now()
        now_ns = now.nanoseconds

        # 第一帧：只记录时间
        if self.last_time_ns is None:
            self.last_time_ns = now_ns
            return

        dt = (now_ns - self.last_time_ns) * 1e-9
        if dt <= 0.0:
            self.get_logger().warn(f'Non-positive dt: {dt:.6f}, skip this frame.')
            return
        self.last_time_ns = now_ns

        # 读取速度 & 转角
        v = float(msg.velocity)          # m/s
        steering = float(msg.steering)   # 弧度

        # Ackermann 模型计算角速度
        if abs(steering) > 1e-6:
            omega = v * math.tan(steering) / self.wheelbase
        else:
            omega = 0.0

        # 积分更新位姿
        # 原来的逻辑
        # self.x += v * math.cos(self.yaw) * dt
        # self.y += v * math.sin(self.yaw) * dt
        # self.yaw += omega * dt

        # 改进的中值积分
        half_yaw = self.yaw + omega * dt * 0.5
        self.x += v * math.cos(half_yaw) * dt
        self.y += v * math.sin(half_yaw) * dt
        self.yaw += omega * dt
        
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # 生成里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()          # ✅ 使用 ROS 当前时间
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # 位姿
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

        # 速度（在 base_link 下）
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = omega

        # （可选）协方差
        odom_msg.pose.covariance = [
            0.05, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.05, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.1, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.1, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 0.1
        ]
        odom_msg.twist.covariance = [0.0] * 36

        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

