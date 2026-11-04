import os
import math
import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class PartolNode(BasicNavigator):
    def __init__(self, node_name='partol_node'):
        super().__init__(node_name)

        # 参数
        self.declare_parameter('initial_points', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.declare_parameter('img_save_path', '/tmp/partol_imgs/')
        self.declare_parameter('loop_patrol', True)
        self.declare_parameter('arrive_tolerance', 0.35)
        self.declare_parameter('photo_wait_sec', 0.5)

        # 读取参数
        self.initial_pose_ = self.get_parameter('initial_points').value
        self.target_points_ = self.get_parameter('target_points').value
        self.img_save_path = self.get_parameter('img_save_path').value
        self.loop_patrol = bool(self.get_parameter('loop_patrol').value)
        self.arrive_tolerance = float(self.get_parameter('arrive_tolerance').value)
        self.photo_wait_sec = float(self.get_parameter('photo_wait_sec').value)

        # 路径处理
        if self.img_save_path and not self.img_save_path.endswith('/'):
            self.img_save_path += '/'
        if self.img_save_path:
            os.makedirs(self.img_save_path, exist_ok=True)

        # TF 和图像
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        self.cv_bridge_ = CvBridge()
        self.latest_img_ = None
        self.img_sub_ = self.create_subscription(
            Image, '/image_raw', self.img_callback, 10
        )

    def img_callback(self, msg: Image):
        self.latest_img_ = msg

    def get_pose_by_xyyaw(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def init_robot_pose(self):
        self.initial_pose_ = self.get_parameter('initial_points').value
        init_pose = self.get_pose_by_xyyaw(
            self.initial_pose_[0],
            self.initial_pose_[1],
            self.initial_pose_[2]
        )
        self.setInitialPose(init_pose)

        # 这里按你的要求，使用你自己的 lidar_localization
        self.waitUntilNav2Active(
            navigator='bt_navigator',
            localizer='lidar_localization'
        )
        self.get_logger().info('Nav2 与 lidar_localization 已进入 Active 状态')

    def get_target_points(self):
        points = []
        self.target_points_ = self.get_parameter('target_points').value

        if len(self.target_points_) % 3 != 0:
            self.get_logger().error('target_points 长度不是 3 的倍数，请检查配置文件')
            return points

        n = int(len(self.target_points_) / 3)
        for i in range(n):
            x = float(self.target_points_[i * 3])
            y = float(self.target_points_[i * 3 + 1])
            yaw = float(self.target_points_[i * 3 + 2])
            points.append([x, y, yaw])
            self.get_logger().info(
                f'获取目标点{i}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
            )
        return points

    def build_waypoints(self):
        points = self.get_target_points()
        waypoints = []
        for i, (x, y, yaw) in enumerate(points):
            pose = self.get_pose_by_xyyaw(x, y, yaw)
            waypoints.append(pose)
            self.get_logger().info(
                f'构建 waypoint{i}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
            )
        return waypoints

    def get_current_pose(self):
        while rclpy.ok():
            try:
                res = self.buffer_.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time(seconds=0.0),
                    rclpy.time.Duration(seconds=1.0)
                )
                return res.transform
            except Exception as e:
                self.get_logger().warn(f'获取 map->base_link 失败: {str(e)}')
                rclpy.spin_once(self, timeout_sec=0.1)

    def calc_dist_to_pose(self, pose: PoseStamped) -> float:
        tf = self.get_current_pose()
        dx = pose.pose.position.x - tf.translation.x
        dy = pose.pose.position.y - tf.translation.y
        return math.hypot(dx, dy)

    def wait_for_new_image(self, timeout_sec=2.0) -> bool:
        start_ns = self.get_clock().now().nanoseconds
        start_img = self.latest_img_

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_img_ is not None and self.latest_img_ is not start_img:
                return True

            elapsed = (self.get_clock().now().nanoseconds - start_ns) / 1e9
            if elapsed > timeout_sec:
                return self.latest_img_ is not None

    def record_img(self, prefix='img'):
        if self.latest_img_ is None:
            self.get_logger().warn('当前没有收到图像，跳过拍照')
            return False

        tf = self.get_current_pose()
        cv_image = self.cv_bridge_.imgmsg_to_cv2(
            self.latest_img_, desired_encoding='bgr8'
        )

        stamp_ns = self.get_clock().now().nanoseconds
        filename = (
            f'{prefix}_{stamp_ns}_'
            f'{tf.translation.x:.2f}_{tf.translation.y:.2f}.png'
        )
        save_path = os.path.join(self.img_save_path, filename)

        ok = cv2.imwrite(save_path, cv_image)
        if ok:
            self.get_logger().info(f'保存图片成功: {save_path}')
            return True
        else:
            self.get_logger().error(f'保存图片失败: {save_path}')
            return False

    def patrol_once(self) -> bool:
        waypoints = self.build_waypoints()
        if not waypoints:
            self.get_logger().error('没有可执行的 waypoint')
            return False

        self.get_logger().info(f'开始执行巡检，共 {len(waypoints)} 个 waypoint')
        self.followWaypoints(waypoints)

        last_waypoint = -1
        photographed = set()

        while not self.isTaskComplete():
            feedback = self.getFeedback()

            if feedback is not None:
                try:
                    current_waypoint = int(feedback.current_waypoint)

                    if current_waypoint != last_waypoint:
                        last_waypoint = current_waypoint
                        self.get_logger().info(
                            f'当前正在前往第 {current_waypoint} 个 waypoint'
                        )

                    # 当 current_waypoint = k 时，表示正在去第 k 个点
                    # 这时可以检查前一个点是否已经到达，到了就补拍
                    arrived_idx = current_waypoint - 1
                    if (
                        arrived_idx >= 0
                        and arrived_idx < len(waypoints)
                        and arrived_idx not in photographed
                    ):
                        dist = self.calc_dist_to_pose(waypoints[arrived_idx])
                        if dist <= self.arrive_tolerance:
                            self.get_logger().info(
                                f'检测到已到达第 {arrived_idx} 个 waypoint 附近，距离 {dist:.3f} m，准备拍照'
                            )
                            self.wait_for_new_image(timeout_sec=self.photo_wait_sec + 1.0)
                            self.record_img(prefix=f'wp_{arrived_idx}')
                            photographed.add(arrived_idx)

                except Exception as e:
                    self.get_logger().warn(f'处理 waypoint feedback 失败: {str(e)}')

            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.getResult()

        # 最后一个点在任务结束后单独补拍
        if len(waypoints) > 0:
            last_idx = len(waypoints) - 1
            if last_idx not in photographed:
                dist = self.calc_dist_to_pose(waypoints[last_idx])
                if dist <= self.arrive_tolerance:
                    self.get_logger().info(
                        f'任务结束，补拍最后一个 waypoint {last_idx}，距离 {dist:.3f} m'
                    )
                    self.wait_for_new_image(timeout_sec=self.photo_wait_sec + 1.0)
                    self.record_img(prefix=f'wp_{last_idx}')
                    photographed.add(last_idx)

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('本轮巡检完成')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('本轮巡检被取消')
            return False
        else:
            self.get_logger().error(f'本轮巡检失败，Nav2 result: {result}')
            return False


def main():
    rclpy.init()
    partol = PartolNode()
    partol.init_robot_pose()

    try:
        while rclpy.ok():
            ok = partol.patrol_once()

            if not ok:
                partol.get_logger().warn('本轮巡检失败')

            if not partol.loop_patrol:
                partol.get_logger().info('loop_patrol=False，执行一轮后退出')
                break

    except KeyboardInterrupt:
        partol.get_logger().info('收到 Ctrl+C，准备退出')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

