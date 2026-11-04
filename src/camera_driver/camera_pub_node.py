import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import time


class SimpleCameraPublisher(Node):
    def __init__(self, name, timer_period):
        super().__init__(name)
        
        # 创建发布器和定时器
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 打开摄像头
        self.cap = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()
        self.frame_id = "camera_link"
        
        # 设置摄像头参数
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)  #  1280    1920
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)  #  720    1080
        self.cap.set(cv2.CAP_PROP_FPS, 20)
        
        if not self.cap.isOpened():
            self.get_logger().error("摄像头打开失败！")
        else:
            # 获取并打印实际参数
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            # 只打印帧率和分辨率
            self.get_logger().info(f"摄像头已打开")
            self.get_logger().info(f"分辨率: {actual_width}x{actual_height}")
            self.get_logger().info(f"帧率: {actual_fps:.1f} Hz")
            
            # 帧率统计变量
            self.frame_count = 0
            self.last_stat_time = time.time()

    def timer_callback(self):
        """读取并发布图像"""
        ret, frame = self.cap.read()
        # 2. 读到数据后立刻取当前时间
        capture_time = self.get_clock().now().to_msg()
        if not ret:
            self.get_logger().warning("读取图像失败", throttle_duration_sec=2.0)
            return
        
        # 帧率统计（每5秒打印一次）
        self.frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.last_stat_time
        
        if elapsed >= 5.0:
            actual_fps = self.frame_count / elapsed
            self.get_logger().info(f"实际运行帧率: {actual_fps:.1f} Hz")
            self.frame_count = 0
            self.last_stat_time = current_time
        
        try:
            # 统一转换为BGR格式
            if frame.ndim == 3 and frame.shape[2] == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_UYVY)
            elif frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            
            # 创建并发布消息
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
            img_msg.header.stamp = capture_time
            img_msg.header.frame_id = self.frame_id
            self.publisher_.publish(img_msg)
                
        except Exception as e:
            self.get_logger().error(f'发布失败: {e}')

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # 设置频率
    desired_fps = 20.0
    if len(sys.argv) > 1:
        try:
            desired_fps = float(sys.argv[1])
        except ValueError:
            print("请提供有效的频率值")
    
    timer_period = 1.0 / desired_fps
    node = SimpleCameraPublisher("camera_publisher", timer_period)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
