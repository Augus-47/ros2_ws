#!/usr/bin/env python3
import os
import time
import threading
import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from nav_msgs.msg import OccupancyGrid


class MapWorkflow(Node):
    def __init__(self):
        super().__init__('map_workflow')

        self.declare_parameter('pcd_file', '/home/nvidia/ros2_ws/maps/map.pcd')
        self.declare_parameter('pgm_file', '/home/nvidia/ros2_ws/maps/map.pgm')
        self.declare_parameter('yaml_file', '/home/nvidia/ros2_ws/maps/map.yaml')
        self.declare_parameter('map_name', '/home/nvidia/ros2_ws/maps/map')
        self.declare_parameter('pcd2pgm_package', 'pcd2pgm')
        self.declare_parameter('pcd2pgm_launch', 'pcd2pgm.launch.py')
        self.declare_parameter('map_topic', '/map')

        self.pcd_file = self.get_parameter('pcd_file').value
        self.pgm_file = self.get_parameter('pgm_file').value
        self.yaml_file = self.get_parameter('yaml_file').value
        self.map_name = self.get_parameter('map_name').value
        self.pcd2pgm_package = self.get_parameter('pcd2pgm_package').value
        self.pcd2pgm_launch = self.get_parameter('pcd2pgm_launch').value
        self.map_topic = self.get_parameter('map_topic').value

        # 确保输出目录存在
        self.output_dir = os.path.dirname(self.map_name)
        if self.output_dir:
            os.makedirs(self.output_dir, exist_ok=True)

        self.map_received = False
        self.workflow_started = False
        self.pcd2pgm_proc = None
        self.state_lock = threading.Lock()

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        self.map_save_cli = self.create_client(Empty, '/map_save')

        self.finish_srv = self.create_service(
            Trigger,
            '/finish_mapping',
            self.finish_mapping_callback
        )

        self.get_logger().info('等待 /map_save 服务...')
        while not self.map_save_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/map_save 服务未就绪，继续等待...')

        self.get_logger().info('/map_save 服务已就绪')
        self.get_logger().info(f'PCD 输出路径: {self.pcd_file}')
        self.get_logger().info(f'PGM 输出路径: {self.pgm_file}')
        self.get_logger().info(f'YAML 输出路径: {self.yaml_file}')
        self.get_logger().info('建图中...')
        self.get_logger().info('建图完成后，请在另一个终端执行：')
        self.get_logger().info('ros2 service call /finish_mapping std_srvs/srv/Trigger "{}"')

    def map_callback(self, msg):
        if msg is not None and msg.info.width > 0 and msg.info.height > 0:
            if not self.map_received:
                self.get_logger().info(
                    f'已收到地图话题 {self.map_topic}: {msg.info.width} x {msg.info.height}'
                )
            self.map_received = True

    def finish_mapping_callback(self, request, response):
        del request

        with self.state_lock:
            if self.workflow_started:
                response.success = False
                response.message = '流程已经开始执行，请勿重复触发'
                return response

            self.workflow_started = True

        response.success = True
        response.message = '已收到结束建图指令，开始保存地图'

        threading.Thread(target=self.run_workflow, daemon=True).start()
        return response

    def call_map_save(self, timeout=20.0):
        self.get_logger().info('调用 /map_save 保存 pcd ...')

        req = Empty.Request()
        future = self.map_save_cli.call_async(req)

        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error('/map_save 调用超时')
                return False
            time.sleep(0.1)

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'/map_save 调用异常: {e}')
            return False

        if result is None:
            self.get_logger().error('/map_save 返回空结果')
            return False

        self.get_logger().info('/map_save 调用成功')
        return True

    def wait_file_ready(self, file_path, timeout=30.0, stable_checks=3):
        self.get_logger().info(f'等待文件生成: {file_path}')

        start_time = time.time()
        last_size = -1
        stable_count = 0

        while time.time() - start_time < timeout:
            if os.path.exists(file_path):
                size = os.path.getsize(file_path)
                if size > 0:
                    if size == last_size:
                        stable_count += 1
                    else:
                        stable_count = 0

                    last_size = size

                    if stable_count >= stable_checks:
                        self.get_logger().info(f'文件已生成并稳定: {file_path}, size={size}')
                        return True

            time.sleep(0.5)

        self.get_logger().error(f'等待文件超时: {file_path}')
        return False

    def start_pcd2pgm(self):
        self.get_logger().info('启动 pcd2pgm...')

        try:
            self.pcd2pgm_proc = subprocess.Popen(
                [
                    'ros2', 'launch',
                    self.pcd2pgm_package,
                    self.pcd2pgm_launch
                ]
            )
            self.get_logger().info(f'pcd2pgm 已启动, pid={self.pcd2pgm_proc.pid}')
            return True
        except Exception as e:
            self.get_logger().error(f'启动 pcd2pgm 失败: {e}')
            return False

    def wait_map_topic_ready(self, timeout=20.0):
        self.map_received = False
        self.get_logger().info(f'等待地图话题 {self.map_topic} ...')

        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.map_received:
                self.get_logger().info(f'地图话题 {self.map_topic} 已就绪')
                return True
            time.sleep(0.2)

        self.get_logger().error(f'等待地图话题超时: {self.map_topic}')
        return False

    def save_pgm(self):
        self.get_logger().info('开始保存 pgm/yaml ...')
        self.get_logger().info(f'map_saver_cli 输出前缀: {self.map_name}')

        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self.map_name],
            capture_output=True,
            text=True
        )

        if result.stdout:
            self.get_logger().info(result.stdout.strip())

        if result.returncode != 0:
            if result.stderr:
                self.get_logger().error(result.stderr.strip())
            else:
                self.get_logger().error('map_saver_cli 执行失败，但没有 stderr 输出')
            return False

        self.get_logger().info('map_saver_cli 执行成功')
        return True

    def cleanup(self):
        if self.pcd2pgm_proc is not None:
            self.get_logger().info('关闭 pcd2pgm 进程')
            self.pcd2pgm_proc.terminate()
            try:
                self.pcd2pgm_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('pcd2pgm 进程未及时退出，执行 kill')
                self.pcd2pgm_proc.kill()
                self.pcd2pgm_proc.wait()

            self.pcd2pgm_proc = None

    def run_workflow(self):
        try:
            self.get_logger().info('收到手动结束信号，开始后处理流程')

            # 1. 保存 pcd
            if not self.call_map_save(timeout=20.0):
                self.get_logger().error('保存 pcd 失败')
                return

            # 2. 等待 pcd 文件稳定
            if not self.wait_file_ready(self.pcd_file, timeout=30.0):
                self.get_logger().error(f'pcd 文件未正确生成: {self.pcd_file}')
                return

            # 3. 启动 pcd2pgm
            if not self.start_pcd2pgm():
                self.get_logger().error('pcd2pgm 启动失败')
                return

            # 4. 等待 /map
            if not self.wait_map_topic_ready(timeout=20.0):
                self.get_logger().error('未收到 OccupancyGrid 地图')
                return

            # 5. 保存 pgm/yaml
            if not self.save_pgm():
                self.get_logger().error('保存 pgm 失败')
                return

            # 6. 检查 pgm/yaml
            pgm_ok = self.wait_file_ready(self.pgm_file, timeout=20.0)
            yaml_ok = self.wait_file_ready(self.yaml_file, timeout=20.0)

            if pgm_ok and yaml_ok:
                self.get_logger().info('全部完成：pcd / pgm / yaml 已生成')
            else:
                self.get_logger().error('最终地图文件不完整')

        except Exception as e:
            self.get_logger().error(f'run_workflow 异常: {e}')
        finally:
            self.cleanup()
            with self.state_lock:
                self.workflow_started = False

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapWorkflow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

