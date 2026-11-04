#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from car_msg.msg import OrinPose

# 直接用 /orin_pose 当作“当前准确位姿接口” 来采巡检点

def yaw_from_quat(q):
    siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class P(Node):
    def __init__(self):
        super().__init__('print_orin_xyyaw_once')
        self.create_subscription(OrinPose, '/orin_pose', self.cb, 10)

    def cb(self, msg: OrinPose):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = yaw_from_quat(q)
        print(f"x={p.x:.3f}, y={p.y:.3f}, yaw(rad)={yaw:.3f}, yaw(deg)={math.degrees(yaw):.1f}")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = P()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

