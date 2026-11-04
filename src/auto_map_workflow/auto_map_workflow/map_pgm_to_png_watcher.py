#!/usr/bin/env python3
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from PIL import Image


class PgmWatcherNode(Node):
    def __init__(self):
        super().__init__('map_pgm_to_png_watcher')

        self.declare_parameter('watch_dir', '/home/nvidia/ros2_ws/maps')
        self.declare_parameter('check_interval', 1.0)
        self.declare_parameter('stable_checks', 3)

        self.watch_dir = Path(self.get_parameter('watch_dir').value)
        self.check_interval = float(self.get_parameter('check_interval').value)
        self.stable_checks = int(self.get_parameter('stable_checks').value)

        self.file_states = {}
        self.converted_files = set()

        self.watch_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Watching directory: {self.watch_dir}')

        self.timer = self.create_timer(self.check_interval, self.scan_and_convert)

    def is_pgm_file_ready(self, file_path: Path) -> bool:
        try:
            size = file_path.stat().st_size
        except FileNotFoundError:
            return False

        if size <= 0:
            return False

        state = self.file_states.get(str(file_path), {"last_size": -1, "stable_count": 0})

        if size == state["last_size"]:
            state["stable_count"] += 1
        else:
            state["stable_count"] = 0

        state["last_size"] = size
        self.file_states[str(file_path)] = state

        return state["stable_count"] >= self.stable_checks

    def convert_pgm_to_png(self, pgm_path: Path) -> bool:
        png_path = pgm_path.with_suffix('.png')

        try:
            with Image.open(pgm_path) as img:
                img = img.convert('L')
                img.save(png_path, format='PNG')
            self.get_logger().info(f'Converted: {pgm_path} -> {png_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to convert {pgm_path}: {e}')
            return False

    def scan_and_convert(self):
        pgm_files = sorted(self.watch_dir.glob('*.pgm'))

        for pgm_file in pgm_files:
            pgm_str = str(pgm_file)

            if pgm_str in self.converted_files:
                continue

            if self.is_pgm_file_ready(pgm_file):
                if self.convert_pgm_to_png(pgm_file):
                    self.converted_files.add(pgm_str)


def main(args=None):
    rclpy.init(args=args)
    node = PgmWatcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

