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

        # ---------------------------
        # 参数
        # ---------------------------
        self.declare_parameter('use_fixed_initial_pose', False)
        self.declare_parameter('initial_points', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'target_points',
            [0.0, 0.0, 0.0, 1.0, 1.0, 1.57]
        )

        self.declare_parameter('img_save_path', '/tmp/partol_imgs/')
        self.declare_parameter('loop_patrol', True)

        # 到点判定（拍照判定）
        self.declare_parameter('arrive_tolerance', 0.12)         # m
        self.declare_parameter('yaw_arrive_tolerance', 0.18)     # rad
        self.declare_parameter('photo_wait_sec', 0.5)
        self.declare_parameter('settle_before_photo_sec', 0.8)
        self.declare_parameter('photo_pose_match_timeout_sec', 1.5)

        # 定位稳定相关
        self.declare_parameter('wait_for_localization_stable', True)
        self.declare_parameter('tf_wait_timeout_sec', 15.0)
        self.declare_parameter('localization_timeout_sec', 20.0)
        self.declare_parameter('stable_window_sec', 2.0)
        self.declare_parameter('stable_check_dt', 0.1)
        self.declare_parameter('stable_position_tolerance', 0.03)
        self.declare_parameter('stable_yaw_tolerance', 0.05)

        # 失败时是否继续下一个点
        self.declare_parameter('continue_on_waypoint_failure', True)

        # ---------------------------
        # 读取参数
        # ---------------------------
        self.use_fixed_initial_pose = bool(
            self.get_parameter('use_fixed_initial_pose').value
        )
        self.initial_pose_ = self.get_parameter('initial_points').value
        self.target_points_ = self.get_parameter('target_points').value

        self.img_save_path = self.get_parameter('img_save_path').value
        self.loop_patrol = bool(self.get_parameter('loop_patrol').value)

        self.arrive_tolerance = float(
            self.get_parameter('arrive_tolerance').value
        )
        self.yaw_arrive_tolerance = float(
            self.get_parameter('yaw_arrive_tolerance').value
        )
        self.photo_wait_sec = float(
            self.get_parameter('photo_wait_sec').value
        )
        self.settle_before_photo_sec = float(
            self.get_parameter('settle_before_photo_sec').value
        )
        self.photo_pose_match_timeout_sec = float(
            self.get_parameter('photo_pose_match_timeout_sec').value
        )

        self.wait_for_localization_stable_ = bool(
            self.get_parameter('wait_for_localization_stable').value
        )
        self.tf_wait_timeout_sec_ = float(
            self.get_parameter('tf_wait_timeout_sec').value
        )
        self.localization_timeout_sec_ = float(
            self.get_parameter('localization_timeout_sec').value
        )
        self.stable_window_sec_ = float(
            self.get_parameter('stable_window_sec').value
        )
        self.stable_check_dt_ = float(
            self.get_parameter('stable_check_dt').value
        )
        self.stable_position_tolerance_ = float(
            self.get_parameter('stable_position_tolerance').value
        )
        self.stable_yaw_tolerance_ = float(
            self.get_parameter('stable_yaw_tolerance').value
        )

        self.continue_on_waypoint_failure = bool(
            self.get_parameter('continue_on_waypoint_failure').value
        )

        # ---------------------------
        # 路径处理
        # ---------------------------
        if self.img_save_path and not self.img_save_path.endswith('/'):
            self.img_save_path += '/'
        if self.img_save_path:
            os.makedirs(self.img_save_path, exist_ok=True)

        # ---------------------------
        # TF 和图像
        # ---------------------------
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        self.cv_bridge_ = CvBridge()
        self.latest_img_ = None
        self.img_sub_ = self.create_subscription(
            Image, '/image_raw', self.img_callback, 10
        )

    # ===========================
    # 图像相关
    # ===========================
    def img_callback(self, msg: Image):
        self.latest_img_ = msg

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

        return False

    def record_img(self, prefix='img'):
        if self.latest_img_ is None:
            self.get_logger().warn('当前没有收到图像，跳过拍照')
            return None

        cur = self.get_current_xyyaw(timeout_sec=1.0)
        if cur is None:
            self.get_logger().warn('获取当前位姿失败，跳过拍照')
            return None

        x, y, yaw = cur

        cv_image = self.cv_bridge_.imgmsg_to_cv2(
            self.latest_img_, desired_encoding='bgr8'
        )

        stamp_ns = self.get_clock().now().nanoseconds
        filename = (
            f'{prefix}_{stamp_ns}_'
            f'x_{x:.2f}_y_{y:.2f}_yaw_{yaw:.2f}.png'
        )
        save_path = os.path.join(self.img_save_path, filename)

        ok = cv2.imwrite(save_path, cv_image)
        if ok:
            self.get_logger().info(f'保存图片成功: {save_path}')
            return save_path
        else:
            self.get_logger().error(f'保存图片失败: {save_path}')
            return None

    # ===========================
    # 位姿/角度工具
    # ===========================
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

    def quat_to_yaw(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, ang: float) -> float:
        while ang > math.pi:
            ang -= 2.0 * math.pi
        while ang < -math.pi:
            ang += 2.0 * math.pi
        return ang

    def angle_span(self, angles) -> float:
        if not angles:
            return 0.0

        best_span = float('inf')
        for ref in angles:
            shifted = [self.normalize_angle(a - ref) for a in angles]
            span = max(shifted) - min(shifted)
            if span < best_span:
                best_span = span
        return best_span

    def format_pose_str(self, x: float, y: float, yaw: float) -> str:
        return (
            f'x={x:.3f}, y={y:.3f}, '
            f'yaw={yaw:.3f} rad ({math.degrees(yaw):.1f} deg)'
        )

    # ===========================
    # TF / 当前位姿
    # ===========================
    def get_current_pose(self, timeout_sec=1.0):
        start_ns = self.get_clock().now().nanoseconds

        while rclpy.ok():
            try:
                res = self.buffer_.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time(seconds=0.0),
                    rclpy.time.Duration(seconds=0.2)
                )
                return res.transform
            except Exception as e:
                elapsed = (self.get_clock().now().nanoseconds - start_ns) / 1e9
                if elapsed > timeout_sec:
                    self.get_logger().warn(f'获取 map->base_link 超时: {str(e)}')
                    return None
                rclpy.spin_once(self, timeout_sec=0.05)

        return None

    def get_current_xyyaw(self, timeout_sec=1.0):
        tf = self.get_current_pose(timeout_sec=timeout_sec)
        if tf is None:
            return None
        x = tf.translation.x
        y = tf.translation.y
        yaw = self.quat_to_yaw(tf.rotation)
        return x, y, yaw

    def wait_for_tf_ready(self, timeout_sec=10.0) -> bool:
        self.get_logger().info('等待 map->base_link TF 可用...')
        start_ns = self.get_clock().now().nanoseconds

        while rclpy.ok():
            cur = self.get_current_xyyaw(timeout_sec=0.3)
            if cur is not None:
                self.get_logger().info(
                    f'map->base_link TF 已可用，当前位置: {self.format_pose_str(*cur)}'
                )
                return True

            elapsed = (self.get_clock().now().nanoseconds - start_ns) / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error(
                    f'等待 map->base_link TF 超时 {timeout_sec:.1f}s'
                )
                return False

            rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def wait_localization_stable(self) -> bool:
        self.get_logger().info('等待定位稳定...')

        need_n = max(3, int(self.stable_window_sec_ / self.stable_check_dt_))
        samples = []

        start_ns = self.get_clock().now().nanoseconds
        while rclpy.ok():
            cur = self.get_current_xyyaw(timeout_sec=0.3)
            if cur is not None:
                x, y, yaw = cur
                samples.append((x, y, yaw))
                if len(samples) > need_n:
                    samples.pop(0)

                if len(samples) == need_n:
                    xs = [s[0] for s in samples]
                    ys = [s[1] for s in samples]
                    yaws = [s[2] for s in samples]

                    dx = max(xs) - min(xs)
                    dy = max(ys) - min(ys)
                    dyaw = self.angle_span(yaws)

                    self.get_logger().info(
                        f'定位稳定性检查: '
                        f'dx={dx:.3f}, dy={dy:.3f}, dyaw={dyaw:.3f}'
                    )

                    if (
                        dx <= self.stable_position_tolerance_
                        and dy <= self.stable_position_tolerance_
                        and dyaw <= self.stable_yaw_tolerance_
                    ):
                        self.get_logger().info(
                            f'定位已稳定，当前位姿: {self.format_pose_str(x, y, yaw)}'
                        )
                        return True

            elapsed = (self.get_clock().now().nanoseconds - start_ns) / 1e9
            if elapsed > self.localization_timeout_sec_:
                self.get_logger().warn(
                    f'等待定位稳定超时 {self.localization_timeout_sec_:.1f}s，'
                    '将继续执行巡检'
                )
                return False

            rclpy.spin_once(self, timeout_sec=self.stable_check_dt_)

        return False

    def spin_for_duration(self, duration_sec: float):
        start_ns = self.get_clock().now().nanoseconds
        while rclpy.ok():
            elapsed = (self.get_clock().now().nanoseconds - start_ns) / 1e9
            if elapsed >= duration_sec:
                break
            rclpy.spin_once(self, timeout_sec=0.05)

    # ===========================
    # 初始化逻辑
    # ===========================
    def init_robot_pose(self):
        if self.use_fixed_initial_pose:
            self.initial_pose_ = self.get_parameter('initial_points').value
            init_pose = self.get_pose_by_xyyaw(
                self.initial_pose_[0],
                self.initial_pose_[1],
                self.initial_pose_[2]
            )
            self.setInitialPose(init_pose)
            self.get_logger().info(
                f'使用固定初始位姿: '
                f'x={self.initial_pose_[0]:.3f}, '
                f'y={self.initial_pose_[1]:.3f}, '
                f'yaw={self.initial_pose_[2]:.3f}'
            )
        else:
            self.get_logger().info(
                'use_fixed_initial_pose=False，不再强制设置初始位姿，'
                '将等待定位模块自行收敛'
            )

        self.waitUntilNav2Active(
            navigator='bt_navigator',
            localizer='lidar_localization'
        )
        self.get_logger().info('Nav2 与 lidar_localization 已进入 Active 状态')

        if not self.wait_for_tf_ready(self.tf_wait_timeout_sec_):
            self.get_logger().warn('TF 未正常就绪，后续巡检可能失败')

        if self.wait_for_localization_stable_:
            self.wait_localization_stable()

    # ===========================
    # waypoint 构建
    # ===========================
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

    # ===========================
    # 位姿误差
    # ===========================
    def calc_pose_error(self, target_pose: PoseStamped, timeout_sec=1.0):
        cur = self.get_current_xyyaw(timeout_sec=timeout_sec)
        if cur is None:
            return None

        cur_x, cur_y, cur_yaw = cur
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        target_yaw = self.quat_to_yaw(target_pose.pose.orientation)

        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)
        dyaw = abs(self.normalize_angle(target_yaw - cur_yaw))

        return {
            'cur_x': cur_x,
            'cur_y': cur_y,
            'cur_yaw': cur_yaw,
            'target_x': target_x,
            'target_y': target_y,
            'target_yaw': target_yaw,
            'dist': dist,
            'dyaw': dyaw
        }

    def wait_until_pose_match_for_photo(self, target_pose: PoseStamped):
        start_ns = self.get_clock().now().nanoseconds
        last_info = None

        while rclpy.ok():
            info = self.calc_pose_error(target_pose, timeout_sec=0.3)
            if info is not None:
                last_info = info
                if (
                    info['dist'] <= self.arrive_tolerance
                    and info['dyaw'] <= self.yaw_arrive_tolerance
                ):
                    return True, info

            elapsed = (self.get_clock().now().nanoseconds - start_ns) / 1e9
            if elapsed > self.photo_pose_match_timeout_sec:
                return False, last_info

            rclpy.spin_once(self, timeout_sec=0.05)

        return False, last_info

    # ===========================
    # 单点导航 + 单点拍照
    # ===========================
    def go_to_waypoint_and_photo(self, idx: int, waypoint: PoseStamped) -> bool:
        target_x = waypoint.pose.position.x
        target_y = waypoint.pose.position.y
        target_yaw = self.quat_to_yaw(waypoint.pose.orientation)

        self.get_logger().info(
            f'========== 开始前往 waypoint {idx} =========='
        )
        self.get_logger().info(
            f'目标位姿: {self.format_pose_str(target_x, target_y, target_yaw)}'
        )

        self.goToPose(waypoint)

        while not self.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'waypoint {idx} 导航成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f'waypoint {idx} 导航被取消')
            return False
        else:
            self.get_logger().error(f'waypoint {idx} 导航失败，Nav2 result: {result}')
            return False

        # 等车停稳一点
        self.spin_for_duration(self.settle_before_photo_sec)

        matched, info = self.wait_until_pose_match_for_photo(waypoint)

        if info is None:
            self.get_logger().error(f'waypoint {idx} 获取实际位姿失败，无法拍照')
            return False

        self.get_logger().info(
            f'waypoint {idx} 实际位姿: '
            f'{self.format_pose_str(info["cur_x"], info["cur_y"], info["cur_yaw"])}'
        )
        self.get_logger().info(
            f'waypoint {idx} 位姿误差: '
            f'dist={info["dist"]:.3f} m, '
            f'dyaw={info["dyaw"]:.3f} rad ({math.degrees(info["dyaw"]):.1f} deg)'
        )

        if not matched:
            self.get_logger().warn(
                f'waypoint {idx} 未达到拍照阈值，'
                f'不拍照，避免生成冗余照片'
            )
            return False

        self.get_logger().info(
            f'waypoint {idx} 已满足拍照条件，准备拍照'
        )

        self.wait_for_new_image(timeout_sec=self.photo_wait_sec + 1.0)
        save_path = self.record_img(prefix=f'wp_{idx}')

        if save_path is not None:
            self.get_logger().info(
                f'waypoint {idx} 拍照成功，当前位置: '
                f'{self.format_pose_str(info["cur_x"], info["cur_y"], info["cur_yaw"])}'
            )
            return True
        else:
            self.get_logger().error(f'waypoint {idx} 拍照失败')
            return False

    # ===========================
    # 巡检主流程
    # ===========================
    def patrol_once(self) -> bool:
        waypoints = self.build_waypoints()
        if not waypoints:
            self.get_logger().error('没有可执行的 waypoint')
            return False

        self.get_logger().info(f'开始执行巡检，共 {len(waypoints)} 个 waypoint')

        overall_ok = True
        success_count = 0

        for idx, waypoint in enumerate(waypoints):
            ok = self.go_to_waypoint_and_photo(idx, waypoint)

            if ok:
                success_count += 1
            else:
                overall_ok = False
                self.get_logger().warn(
                    f'waypoint {idx} 未完成“到点并拍照”'
                )
                if not self.continue_on_waypoint_failure:
                    self.get_logger().warn(
                        'continue_on_waypoint_failure=False，提前结束本轮巡检'
                    )
                    break

        self.get_logger().info(
            f'本轮巡检结束: 成功拍照 {success_count}/{len(waypoints)}'
        )

        if overall_ok:
            self.get_logger().info('本轮巡检完成，所有巡检点均已到点并拍照成功')
        else:
            self.get_logger().warn('本轮巡检完成，但存在未成功到点拍照的巡检点')

        return overall_ok


def main():
    rclpy.init()
    partol = PartolNode()
    partol.init_robot_pose()

    try:
        while rclpy.ok():
            ok = partol.patrol_once()

            if not ok:
                partol.get_logger().warn('本轮巡检存在失败点')

            if not partol.loop_patrol:
                partol.get_logger().info('loop_patrol=False，执行一轮后退出')
                break

    except KeyboardInterrupt:
        partol.get_logger().info('收到 Ctrl+C，准备退出')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
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

        # ---------------------------
        # 参数
        # ---------------------------
        self.declare_parameter('use_fixed_initial_pose', False)
        self.declare_parameter('initial_points', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'target_points',
            [0.0, 0.0, 0.0, 1.0, 1.0, 1.57]
        )

        self.declare_parameter('img_save_path', '/tmp/partol_imgs/')
        self.declare_parameter('loop_patrol', True)

        # 到点判定（拍照判定）
        self.declare_parameter('arrive_tolerance', 0.12)         # m
        self.declare_parameter('yaw_arrive_tolerance', 0.18)     # rad
        self.declare_parameter('photo_wait_sec', 0.5)
        self.declare_parameter('settle_before_photo_sec', 0.8)
        self.declare_parameter('photo_pose_match_timeout_sec', 1.5)

        # 定位稳定相关
        self.declare_parameter('wait_for_localization_stable', True)
        self.declare_parameter('tf_wait_timeout_sec', 15.0)
        self.declare_parameter('localization_timeout_sec', 20.0)
        self.declare_parameter('stable_window_sec', 2.0)
        self.declare_parameter('stable_check_dt', 0.1)
        self.declare_parameter('stable_position_tolerance', 0.03)
        self.declare_parameter('stable_yaw_tolerance', 0.05)

        # 失败时是否继续下一个点
        self.declare_parameter('continue_on_waypoint_failure', True)

        # ---------------------------
        # 读取参数
        # ---------------------------
        self.use_fixed_initial_pose = bool(
            self.get_parameter('use_fixed_initial_pose').value
        )
        self.initial_pose_ = self.get_parameter('initial_points').value
        self.target_points_ = self.get_parameter('target_points').value

        self.img_save_path = self.get_parameter('img_save_path').value
        self.loop_patrol = bool(self.get_parameter('loop_patrol').value)

        self.arrive_tolerance = float(
            self.get_parameter('arrive_tolerance').value
        )
        self.yaw_arrive_tolerance = float(
            self.get_parameter('yaw_arrive_tolerance').value
        )
        self.photo_wait_sec = float(
            self.get_parameter('photo_wait_sec').value
        )
        self.settle_before_photo_sec = float(
            self.get_parameter('settle_before_photo_sec').value
        )
        self.photo_pose_match_timeout_sec = float(
            self.get_parameter('photo_pose_match_timeout_sec').value
        )

        self.wait_for_localization_stable_ = bool(
            self.get_parameter('wait_for_localization_stable').value
        )
        self.tf_wait_timeout_sec_ = float(
            self.get_parameter('tf_wait_timeout_sec').value
        )
        self.localization_timeout_sec_ = float(
            self.get_parameter('localization_timeout_sec').value
        )
        self.stable_window_sec_ = float(
            self.get_parameter('stable_window_sec').value
        )
        self.stable_check_dt_ = float(
            self.get_parameter('stable_check_dt').value
        )
        self.stable_position_tolerance_ = float(
            self.get_parameter('stable_position_tolerance').value
        )
        self.stable_yaw_tolerance_ = float(
            self.get_parameter('stable_yaw_tolerance').value
        )

        self.continue_on_waypoint_failure = bool(
            self.get_parameter('continue_on_waypoint_failure').value
        )

        # ---------------------------
        # 路径处理
        # ---------------------------
        if self.img_save_path and not self.img_save_path.endswith('/'):
            self.img_save_path += '/'
        if self.img_save_path:
            os.makedirs(self.img_save_path, exist_ok=True)

        # ---------------------------
        # TF 和图像
        # ---------------------------
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)
        self.cv_bridge_ = CvBridge()
        self.latest_img_ = None
        self.img_sub_ = self.create_subscription(
            Image, '/image_raw', self.img_callback, 10
        )

    # ===========================
    # 图像相关
    # ===========================
    def img_callback(self, msg: Image):
        self.latest_img_ = msg

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

        return False

    def record_img(self, prefix='img'):
        if self.latest_img_ is None:
            self.get_logger().warn('当前没有收到图像，跳过拍照')
            return None

        cur = self.get_current_xyyaw(timeout_sec=1.0)
        if cur is None:
            self.get_logger().warn('获取当前位姿失败，跳过拍照')
            return None

        x, y, yaw = cur

        cv_image = self.cv_bridge_.imgmsg_to_cv2(
            self.latest_img_, desired_encoding='bgr8'
        )

        stamp_ns = self.get_clock().now().nanoseconds
        filename = (
            f'{prefix}_{stamp_ns}_'
            f'x_{x:.2f}_y_{y:.2f}_yaw_{yaw:.2f}.png'
        )
        save_path = os.path.join(self.img_save_path, filename)

        ok = cv2.imwrite(save_path, cv_image)
        if ok:
            self.get_logger().info(f'保存图片成功: {save_path}')
            return save_path
        else:
            self.get_logger().error(f'保存图片失败: {save_path}')
            return None

    # ===========================
    # 位姿/角度工具
    # ===========================
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

    def quat_to_yaw(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, ang: float) -> float:
        while ang > math.pi:
            ang -= 2.0 * math.pi
        while ang < -math.pi:
            ang += 2.0 * math.pi
        return ang

    def angle_span(self, angles) -> float:
        if not angles:
            return 0.0

        best_span = float('inf')
        for ref in angles:
            shifted = [self.normalize_angle(a - ref) for a in angles]
            span = max(shifted) - min(shifted)
            if span < best_span:
                best_span = span
        return best_span

    def format_pose_str(self, x: float, y: float, yaw: float) -> str:
        return (
            f'x={x:.3f}, y={y:.3f}, '
            f'yaw={yaw:.3f} rad ({math.degrees(yaw):.1f} deg)'
        )

    # ===========================
    # TF / 当前位姿
    # ===========================
    def get_current_pose(self, timeout_sec=1.0):
        start_ns = self.get_clock().now().nanoseconds

        while rclpy.ok():
            try:
                res = self.buffer_.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time(seconds=0.0),
                    rclpy.time.Duration(seconds=0.2)
                )
                return res.transform
            except Exception as e:
                elapsed = (self.get_clock().now().nanoseconds - start_ns) / 1e9
                if elapsed > timeout_sec:
                    self.get_logger().warn(f'获取 map->base_link 超时: {str(e)}')
                    return None
                rclpy.spin_once(self, timeout_sec=0.05)

        return None

    def get_current_xyyaw(self, timeout_sec=1.0):
        tf = self.get_current_pose(timeout_sec=timeout_sec)
        if tf is None:
            return None
        x = tf.translation.x
        y = tf.translation.y
        yaw = self.quat_to_yaw(tf.rotation)
        return x, y, yaw

    def wait_for_tf_ready(self, timeout_sec=10.0) -> bool:
        self.get_logger().info('等待 map->base_link TF 可用...')
        start_ns = self.get_clock().now().nanoseconds

        while rclpy.ok():
            cur = self.get_current_xyyaw(timeout_sec=0.3)
            if cur is not None:
                self.get_logger().info(
                    f'map->base_link TF 已可用，当前位置: {self.format_pose_str(*cur)}'
                )
                return True

            elapsed = (self.get_clock().now().nanoseconds - start_ns) / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error(
                    f'等待 map->base_link TF 超时 {timeout_sec:.1f}s'
                )
                return False

            rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def wait_localization_stable(self) -> bool:
        self.get_logger().info('等待定位稳定...')

        need_n = max(3, int(self.stable_window_sec_ / self.stable_check_dt_))
        samples = []

        start_ns = self.get_clock().now().nanoseconds
        while rclpy.ok():
            cur = self.get_current_xyyaw(timeout_sec=0.3)
            if cur is not None:
                x, y, yaw = cur
                samples.append((x, y, yaw))
                if len(samples) > need_n:
                    samples.pop(0)

                if len(samples) == need_n:
                    xs = [s[0] for s in samples]
                    ys = [s[1] for s in samples]
                    yaws = [s[2] for s in samples]

                    dx = max(xs) - min(xs)
                    dy = max(ys) - min(ys)
                    dyaw = self.angle_span(yaws)

                    self.get_logger().info(
                        f'定位稳定性检查: '
                        f'dx={dx:.3f}, dy={dy:.3f}, dyaw={dyaw:.3f}'
                    )

                    if (
                        dx <= self.stable_position_tolerance_
                        and dy <= self.stable_position_tolerance_
                        and dyaw <= self.stable_yaw_tolerance_
                    ):
                        self.get_logger().info(
                            f'定位已稳定，当前位姿: {self.format_pose_str(x, y, yaw)}'
                        )
                        return True

            elapsed = (self.get_clock().now().nanoseconds - start_ns) / 1e9
            if elapsed > self.localization_timeout_sec_:
                self.get_logger().warn(
                    f'等待定位稳定超时 {self.localization_timeout_sec_:.1f}s，'
                    '将继续执行巡检'
                )
                return False

            rclpy.spin_once(self, timeout_sec=self.stable_check_dt_)

        return False

    def spin_for_duration(self, duration_sec: float):
        start_ns = self.get_clock().now().nanoseconds
        while rclpy.ok():
            elapsed = (self.get_clock().now().nanoseconds - start_ns) / 1e9
            if elapsed >= duration_sec:
                break
            rclpy.spin_once(self, timeout_sec=0.05)

    # ===========================
    # 初始化逻辑
    # ===========================
    def init_robot_pose(self):
        if self.use_fixed_initial_pose:
            self.initial_pose_ = self.get_parameter('initial_points').value
            init_pose = self.get_pose_by_xyyaw(
                self.initial_pose_[0],
                self.initial_pose_[1],
                self.initial_pose_[2]
            )
            self.setInitialPose(init_pose)
            self.get_logger().info(
                f'使用固定初始位姿: '
                f'x={self.initial_pose_[0]:.3f}, '
                f'y={self.initial_pose_[1]:.3f}, '
                f'yaw={self.initial_pose_[2]:.3f}'
            )
        else:
            self.get_logger().info(
                'use_fixed_initial_pose=False，不再强制设置初始位姿，'
                '将等待定位模块自行收敛'
            )

        self.waitUntilNav2Active(
            navigator='bt_navigator',
            localizer='lidar_localization'
        )
        self.get_logger().info('Nav2 与 lidar_localization 已进入 Active 状态')

        if not self.wait_for_tf_ready(self.tf_wait_timeout_sec_):
            self.get_logger().warn('TF 未正常就绪，后续巡检可能失败')

        if self.wait_for_localization_stable_:
            self.wait_localization_stable()

    # ===========================
    # waypoint 构建
    # ===========================
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

    # ===========================
    # 位姿误差
    # ===========================
    def calc_pose_error(self, target_pose: PoseStamped, timeout_sec=1.0):
        cur = self.get_current_xyyaw(timeout_sec=timeout_sec)
        if cur is None:
            return None

        cur_x, cur_y, cur_yaw = cur
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        target_yaw = self.quat_to_yaw(target_pose.pose.orientation)

        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)
        dyaw = abs(self.normalize_angle(target_yaw - cur_yaw))

        return {
            'cur_x': cur_x,
            'cur_y': cur_y,
            'cur_yaw': cur_yaw,
            'target_x': target_x,
            'target_y': target_y,
            'target_yaw': target_yaw,
            'dist': dist,
            'dyaw': dyaw
        }

    def wait_until_pose_match_for_photo(self, target_pose: PoseStamped):
        start_ns = self.get_clock().now().nanoseconds
        last_info = None

        while rclpy.ok():
            info = self.calc_pose_error(target_pose, timeout_sec=0.3)
            if info is not None:
                last_info = info
                if (
                    info['dist'] <= self.arrive_tolerance
                    and info['dyaw'] <= self.yaw_arrive_tolerance
                ):
                    return True, info

            elapsed = (self.get_clock().now().nanoseconds - start_ns) / 1e9
            if elapsed > self.photo_pose_match_timeout_sec:
                return False, last_info

            rclpy.spin_once(self, timeout_sec=0.05)

        return False, last_info

    # ===========================
    # 单点导航 + 单点拍照
    # ===========================
    def go_to_waypoint_and_photo(self, idx: int, waypoint: PoseStamped) -> bool:
        target_x = waypoint.pose.position.x
        target_y = waypoint.pose.position.y
        target_yaw = self.quat_to_yaw(waypoint.pose.orientation)

        self.get_logger().info(
            f'========== 开始前往 waypoint {idx} =========='
        )
        self.get_logger().info(
            f'目标位姿: {self.format_pose_str(target_x, target_y, target_yaw)}'
        )

        self.goToPose(waypoint)

        while not self.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'waypoint {idx} 导航成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f'waypoint {idx} 导航被取消')
            return False
        else:
            self.get_logger().error(f'waypoint {idx} 导航失败，Nav2 result: {result}')
            return False

        # 等车停稳一点
        self.spin_for_duration(self.settle_before_photo_sec)

        matched, info = self.wait_until_pose_match_for_photo(waypoint)

        if info is None:
            self.get_logger().error(f'waypoint {idx} 获取实际位姿失败，无法拍照')
            return False

        self.get_logger().info(
            f'waypoint {idx} 实际位姿: '
            f'{self.format_pose_str(info["cur_x"], info["cur_y"], info["cur_yaw"])}'
        )
        self.get_logger().info(
            f'waypoint {idx} 位姿误差: '
            f'dist={info["dist"]:.3f} m, '
            f'dyaw={info["dyaw"]:.3f} rad ({math.degrees(info["dyaw"]):.1f} deg)'
        )

        if not matched:
            self.get_logger().warn(
                f'waypoint {idx} 未达到拍照阈值，'
                f'不拍照，避免生成冗余照片'
            )
            return False

        self.get_logger().info(
            f'waypoint {idx} 已满足拍照条件，准备拍照'
        )

        self.wait_for_new_image(timeout_sec=self.photo_wait_sec + 1.0)
        save_path = self.record_img(prefix=f'wp_{idx}')

        if save_path is not None:
            self.get_logger().info(
                f'waypoint {idx} 拍照成功，当前位置: '
                f'{self.format_pose_str(info["cur_x"], info["cur_y"], info["cur_yaw"])}'
            )
            return True
        else:
            self.get_logger().error(f'waypoint {idx} 拍照失败')
            return False

    # ===========================
    # 巡检主流程
    # ===========================
    def patrol_once(self) -> bool:
        waypoints = self.build_waypoints()
        if not waypoints:
            self.get_logger().error('没有可执行的 waypoint')
            return False

        self.get_logger().info(f'开始执行巡检，共 {len(waypoints)} 个 waypoint')

        overall_ok = True
        success_count = 0

        for idx, waypoint in enumerate(waypoints):
            ok = self.go_to_waypoint_and_photo(idx, waypoint)

            if ok:
                success_count += 1
            else:
                overall_ok = False
                self.get_logger().warn(
                    f'waypoint {idx} 未完成“到点并拍照”'
                )
                if not self.continue_on_waypoint_failure:
                    self.get_logger().warn(
                        'continue_on_waypoint_failure=False，提前结束本轮巡检'
                    )
                    break

        self.get_logger().info(
            f'本轮巡检结束: 成功拍照 {success_count}/{len(waypoints)}'
        )

        if overall_ok:
            self.get_logger().info('本轮巡检完成，所有巡检点均已到点并拍照成功')
        else:
            self.get_logger().warn('本轮巡检完成，但存在未成功到点拍照的巡检点')

        return overall_ok


def main():
    rclpy.init()
    partol = PartolNode()
    partol.init_robot_pose()

    try:
        while rclpy.ok():
            ok = partol.patrol_once()

            if not ok:
                partol.get_logger().warn('本轮巡检存在失败点')

            if not partol.loop_patrol:
                partol.get_logger().info('loop_patrol=False，执行一轮后退出')
                break

    except KeyboardInterrupt:
        partol.get_logger().info('收到 Ctrl+C，准备退出')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

