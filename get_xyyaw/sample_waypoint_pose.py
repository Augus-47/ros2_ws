#!/usr/bin/env python3
import math

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class SampleMapBaseLinkPose(Node):
    def __init__(self):
        super().__init__('sample_map_base_link_pose')

        # 可调参数
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('sample_count', 50)         # 采样次数
        self.declare_parameter('sample_period', 0.1)       # 秒，10Hz
        self.declare_parameter('lookup_timeout', 0.2)      # TF 查询超时
        self.declare_parameter('warmup_sec', 2.0)          # 启动后先等几秒，让定位更稳
        self.declare_parameter('warn_dx', 0.03)            # m
        self.declare_parameter('warn_dy', 0.03)            # m
        self.declare_parameter('warn_dyaw_deg', 2.0)       # deg

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.sample_count = int(self.get_parameter('sample_count').value)
        self.sample_period = float(self.get_parameter('sample_period').value)
        self.lookup_timeout = float(self.get_parameter('lookup_timeout').value)
        self.warmup_sec = float(self.get_parameter('warmup_sec').value)
        self.warn_dx = float(self.get_parameter('warn_dx').value)
        self.warn_dy = float(self.get_parameter('warn_dy').value)
        self.warn_dyaw_deg = float(self.get_parameter('warn_dyaw_deg').value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.samples = []
        self.started_sampling = False
        self.start_ns = self.get_clock().now().nanoseconds

        self.get_logger().info(
            f'脚本启动，先等待 {self.warmup_sec:.1f}s，请保持车辆静止...'
        )

        self.timer = self.create_timer(self.sample_period, self.on_timer)

    def on_timer(self):
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self.start_ns) / 1e9

        if not self.started_sampling:
            if elapsed < self.warmup_sec:
                return
            self.started_sampling = True
            self.get_logger().info(
                f'开始采样 {self.target_frame} -> {self.source_frame}，'
                f'共 {self.sample_count} 次，周期 {self.sample_period:.2f}s'
            )

        try:
            tf_msg = self.buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.lookup_timeout)
            )

            tf = tf_msg.transform
            x = tf.translation.x
            y = tf.translation.y
            yaw = yaw_from_quat(tf.rotation)

            self.samples.append((x, y, yaw))

            self.get_logger().info(
                f'[{len(self.samples):02d}/{self.sample_count}] '
                f'x={x:.3f}, y={y:.3f}, '
                f'yaw={yaw:.3f} rad ({math.degrees(yaw):.1f} deg)'
            )

            if len(self.samples) >= self.sample_count:
                self.report_and_shutdown()

        except TransformException as e:
            self.get_logger().warn(
                f'获取 {self.target_frame}->{self.source_frame} 失败: {str(e)}'
            )
        except Exception as e:
            self.get_logger().error(f'未知异常: {str(e)}')

    def report_and_shutdown(self):
        xs = [s[0] for s in self.samples]
        ys = [s[1] for s in self.samples]
        yaws = [s[2] for s in self.samples]

        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)

        # 角度平均
        mean_sin = sum(math.sin(a) for a in yaws) / len(yaws)
        mean_cos = sum(math.cos(a) for a in yaws) / len(yaws)
        mean_yaw = math.atan2(mean_sin, mean_cos)

        max_dx = max(abs(x - mean_x) for x in xs)
        max_dy = max(abs(y - mean_y) for y in ys)
        max_dyaw = max(abs(normalize_angle(a - mean_yaw)) for a in yaws)
        max_dyaw_deg = math.degrees(max_dyaw)

        print('\n================ FINAL WAYPOINT POSE ================')
        print(f'x = {mean_x:.3f}')
        print(f'y = {mean_y:.3f}')
        print(f'yaw(rad) = {mean_yaw:.3f}')
        print(f'yaw(deg) = {math.degrees(mean_yaw):.1f}')
        print('---------------- stability check -------------------')
        print(f'max |dx|   = {max_dx:.3f} m')
        print(f'max |dy|   = {max_dy:.3f} m')
        print(f'max |dyaw| = {max_dyaw_deg:.2f} deg')

        if (
            max_dx > self.warn_dx or
            max_dy > self.warn_dy or
            max_dyaw_deg > self.warn_dyaw_deg
        ):
            print('\n[WARN] 当前点位抖动偏大，建议：')
            print('1. 再等几秒让定位更稳定')
            print('2. 保证车辆完全静止')
            print('3. 重新运行本脚本再采一次')
        else:
            print('\n[OK] 当前点位稳定性可接受，可以作为 waypoint 候选值。')

        print('====================================================\n')

        rclpy.shutdown()


def main():
    rclpy.init()
    node = SampleMapBaseLinkPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，退出。')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

