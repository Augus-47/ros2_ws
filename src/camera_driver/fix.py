#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class CameraDebug(Node):
    def __init__(self):
        super().__init__('camera_debug')
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('摄像头调试器启动...')

    def image_callback(self, msg):
        self.get_logger().info(f'收到图像: {msg.width}x{msg.height}, 编码: {msg.encoding}')
        
        # 检查原始数据
        data_array = np.frombuffer(msg.data, dtype=np.uint8)
        self.get_logger().info(f'数据统计 - 最小值: {data_array.min()}, 最大值: {data_array.max()}, 均值: {data_array.mean():.2f}')
        
        # 尝试不同的转换方法
        self.try_all_conversions(msg)

    def try_all_conversions(self, msg):
        """尝试所有可能的转换方法"""
        conversions = [
            ('bgr8', '直接BGR'),
            ('rgb8', '直接RGB'),
            ('mono8', '灰度'),
        ]
        
        for encoding, description in conversions:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
                self.get_logger().info(f'{description}({encoding}): 形状 {cv_image.shape}')
                
                # 显示图像
                display_image = cv_image
                if encoding == 'rgb8':
                    display_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                elif len(cv_image.shape) == 2:
                    display_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                
                cv2.imshow(f'{description} - {encoding}', display_image)
                cv2.waitKey(1)
                
            except Exception as e:
                self.get_logger().warning(f'{description} 失败: {e}')

def main():
    rclpy.init()
    debug = CameraDebug()
    try:
        rclpy.spin(debug)
    except KeyboardInterrupt:
        pass
    finally:
        debug.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
