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

        # 是否使用固定初始位姿
        self.declare_parameter('use_fixed_initial_pose', False)

        # 初始位姿：仍然按 base_link（后轴中心）定义
        self.declare_parameter('initial_points', [0.0, 0.0, 0.0])

        # 重点：
        # target_points 现在定义为“车头参考点”的目标位姿
        # 格式: [front_x, front_y, front_yaw, front_x2, front_y2, front_yaw2, ...]
        self.declare_parameter(
            'target_points',
            [0.0, 0.0, 0.0, 1.0, 1.0, 1.57]
        )

        self.declare_parameter('img_save_path', '/tmp/partol_imgs/')
        self.declare_parameter('loop_patrol', True)

        # ---------------------------
        # 拍照触发参数（核心）
        # ---------------------------

        # 车头参考点进入该半径范围，就认为“到达拍照区”
        self.declare_parameter('photo_trigger_radius', 0.30)   # m

        # 触发拍照后，取消当前导航任务并等待底盘稳定
        self.declare_parameter('settle_before_photo_sec', 0.8)

        # 等待新图像时间
        self.declare_parameter('photo_wait_sec', 0.5)

        # cancelTask() 后，最多等待多久让当前任务结束
        self.declare_parameter('cancel_wait_sec', 1.0)

        # 车头参考点相对 base_link（后轴中心）的前向距离
        self.declare_parameter('front_offset', 0.8)

        # ---------------------------
        # 定位稳定相关
        # ---------------------------
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

        self.photo_trigger_radius = float(
            self.get_parameter('photo_trigger_radius').value
        )
        self.settle_before_photo_sec = float(
            self.get_parameter('settle_before_photo_sec').value
        )
        self.photo_wait_sec = float(
            self.get_parameter('photo_wait_sec').value
        )
        self.cancel_wait_sec = float(
            self.get_parameter('cancel_wait_sec').value
        )
        self.front_offset = float(
            self.get_parameter('front_offset').value
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
        """
        保存图像。
        文件名记录的是“车头参考点”的位姿，便于后续核对。
        """
        if self.latest_img_ is None:
            self.get_logger().warn('当前没有收到图像，跳过拍照')
            return None

        cur = self.get_current_xyyaw(timeout_sec=1.0)
        if cur is None:
            self.get_logger().warn('获取当前位姿失败，跳过拍照')
            return None

        base_x, base_y, base_yaw = cur
        front_x, front_y, front_yaw = self.get_front_xyyaw_from_base(
            base_x, base_y, base_yaw
        )

        cv_image = self.cv_bridge_.imgmsg_to_cv2(
            self.latest_img_, desired_encoding='bgr8'
        )

        stamp_ns = self.get_clock().now().nanoseconds
        filename = (
            f'{prefix}_{stamp_ns}_'
            f'front_x_{front_x:.2f}_y_{front_y:.2f}_yaw_{front_yaw:.2f}.png'
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

    def get_front_xyyaw_from_base(
        self,
        base_x: float,
        base_y: float,
        base_yaw: float
    ):
        """
        根据 base_link（后轴中心）位姿，计算车头参考点位姿。
        """
        front_x = base_x + self.front_offset * math.cos(base_yaw)
        front_y = base_y + self.front_offset * math.sin(base_yaw)
        front_yaw = base_yaw
        return front_x, front_y, front_yaw

    def get_base_xyyaw_from_front(
        self,
        front_x: float,
        front_y: float,
        front_yaw: float
    ):
        """
        由车头参考点目标位姿，换算出真正发给 Nav2 的 base_link 目标位姿。
        """
        base_x = front_x - self.front_offset * math.cos(front_yaw)
        base_y = front_y - self.front_offset * math.sin(front_yaw)
        base_yaw = front_yaw
        return base_x, base_y, base_yaw

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

    def cancel_current_task_and_wait(self, timeout_sec=1.0) -> bool:
        """
        取消当前导航任务，并等待任务真正结束。
        这样可以避免刚 cancel 就马上发下一个 goal。
        """
        self.cancelTask()

        start_ns = self.get_clock().now().nanoseconds
        while rclpy.ok():
            if self.isTaskComplete():
                return True

            elapsed = (self.get_clock().now().nanoseconds - start_ns) / 1e9
            if elapsed > timeout_sec:
                self.get_logger().warn(
                    f'cancelTask() 后等待任务结束超时 {timeout_sec:.2f}s'
                )
                return False

            rclpy.spin_once(self, timeout_sec=0.05)

        return False

    # ===========================
    # 初始化逻辑
    # ===========================
    def init_robot_pose(self):
        """
        初始位姿仍然按 base_link（后轴中心）设置。
        """
        if self.use_fixed_initial_pose:
            self.initial_pose_ = self.get_parameter('initial_points').value
            init_pose = self.get_pose_by_xyyaw(
                self.initial_pose_[0],
                self.initial_pose_[1],
                self.initial_pose_[2]
            )
            self.setInitialPose(init_pose)
            self.get_logger().info(
                f'使用固定初始位姿(base_link): '
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
    # 目标点处理
    # ===========================
    def get_target_points(self):
        """
        读取业务目标点。
        注意：这里返回的是“车头参考点”的目标位姿。
        """
        points = []
        self.target_points_ = self.get_parameter('target_points').value

        if len(self.target_points_) % 3 != 0:
            self.get_logger().error('target_points 长度不是 3 的倍数，请检查配置文件')
            return points

        n = int(len(self.target_points_) / 3)
        for i in range(n):
            front_x = float(self.target_points_[i * 3])
            front_y = float(self.target_points_[i * 3 + 1])
            front_yaw = float(self.target_points_[i * 3 + 2])

            points.append([front_x, front_y, front_yaw])

            self.get_logger().info(
                f'获取业务目标点{i}(车头参考点): '
                f'x={front_x:.3f}, y={front_y:.3f}, yaw={front_yaw:.3f}'
            )

        return points

    def build_navigation_targets(self, business_points):
        """
        做法2：
        - target_points 是车头参考点目标
        - 这里换算出 base_link（后轴中心）导航目标
        """
        nav_waypoints = []

        for i, (front_x, front_y, front_yaw) in enumerate(business_points):
            base_x, base_y, base_yaw = self.get_base_xyyaw_from_front(
                front_x, front_y, front_yaw
            )
            nav_pose = self.get_pose_by_xyyaw(base_x, base_y, base_yaw)
            nav_waypoints.append(nav_pose)

            self.get_logger().info(
                f'业务目标点{i}(车头参考点): '
                f'{self.format_pose_str(front_x, front_y, front_yaw)}'
            )
            self.get_logger().info(
                f'换算后的导航目标{i}(base_link/后轴中心): '
                f'{self.format_pose_str(base_x, base_y, base_yaw)}'
            )

        return nav_waypoints

    # ===========================
    # 误差计算（以车头参考点为准）
    # ===========================
    def calc_front_pose_error(self, target_pose: PoseStamped, timeout_sec=1.0):
        """
        计算“当前车头参考点”相对于“业务目标点”的误差。
        这里 dist 是核心：
        只要车头进入 photo_trigger_radius，就触发拍照。
        """
        cur = self.get_current_xyyaw(timeout_sec=timeout_sec)
        if cur is None:
            return None

        base_x, base_y, base_yaw = cur
        front_x, front_y, front_yaw = self.get_front_xyyaw_from_base(
            base_x, base_y, base_yaw
        )

        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        target_yaw = self.quat_to_yaw(target_pose.pose.orientation)

        dx = target_x - front_x
        dy = target_y - front_y
        dist = math.hypot(dx, dy)
        dyaw = abs(self.normalize_angle(target_yaw - front_yaw))

        return {
            'base_x': base_x,
            'base_y': base_y,
            'base_yaw': base_yaw,
            'front_x': front_x,
            'front_y': front_y,
            'front_yaw': front_yaw,
            'target_x': target_x,
            'target_y': target_y,
            'target_yaw': target_yaw,
            'dist': dist,
            'dyaw': dyaw
        }

    # ===========================
    # 单点导航 + 到拍照区立即触发拍照
    # ===========================
    def go_to_waypoint_and_photo(
        self,
        idx: int,
        nav_waypoint: PoseStamped,
        business_target_pose: PoseStamped
    ) -> bool:
        """
        核心逻辑：
        1. nav_waypoint 是真正发给 Nav2 的 base_link 导航目标
        2. business_target_pose 是业务目标点（车头参考点）
        3. 导航过程中实时检查车头点
        4. 只要车头进入 photo_trigger_radius，就 cancelTask() 并拍照
        5. 不再死等 Nav2 reached
        """
        nav_x = nav_waypoint.pose.position.x
        nav_y = nav_waypoint.pose.position.y
        nav_yaw = self.quat_to_yaw(nav_waypoint.pose.orientation)

        target_x = business_target_pose.pose.position.x
        target_y = business_target_pose.pose.position.y
        target_yaw = self.quat_to_yaw(business_target_pose.pose.orientation)

        self.get_logger().info(
            f'========== 开始前往 waypoint {idx} =========='
        )
        self.get_logger().info(
            f'业务目标点(车头参考点): '
            f'{self.format_pose_str(target_x, target_y, target_yaw)}'
        )
        self.get_logger().info(
            f'实际导航目标(base_link/后轴中心): '
            f'{self.format_pose_str(nav_x, nav_y, nav_yaw)}'
        )
        self.get_logger().info(
            f'进入拍照区半径: {self.photo_trigger_radius:.3f} m'
        )
        self.get_logger().info(
            f'front_offset: {self.front_offset:.3f} m'
        )

        self.goToPose(nav_waypoint)

        reached_photo_zone = False
        last_info = None

        while rclpy.ok():
            info = self.calc_front_pose_error(business_target_pose, timeout_sec=0.2)
            if info is not None:
                last_info = info

                # 这里是本版本的核心：
                # 只要“车头参考点”进入拍照半径，就立即触发拍照逻辑
                if info['dist'] <= self.photo_trigger_radius:
                    reached_photo_zone = True
                    self.get_logger().info(
                        f'waypoint {idx} 车头已进入拍照区，'
                        f'dist={info["dist"]:.3f} m，取消当前导航任务并准备拍照'
                    )
                    self.cancel_current_task_and_wait(self.cancel_wait_sec)
                    break

            # 如果 Nav2 自己先结束了，也跳出
            if self.isTaskComplete():
                break

            rclpy.spin_once(self, timeout_sec=0.05)

        # 再做一次最终检查，防止刚好边界时错过
        final_info = self.calc_front_pose_error(business_target_pose, timeout_sec=0.5)
        if final_info is not None:
            last_info = final_info
            if final_info['dist'] <= self.photo_trigger_radius:
                reached_photo_zone = True

        if last_info is not None:
            self.get_logger().info(
                f'waypoint {idx} 当前 base_link 位姿: '
                f'{self.format_pose_str(last_info["base_x"], last_info["base_y"], last_info["base_yaw"])}'
            )
            self.get_logger().info(
                f'waypoint {idx} 当前车头参考点位姿: '
                f'{self.format_pose_str(last_info["front_x"], last_info["front_y"], last_info["front_yaw"])}'
            )
            self.get_logger().info(
                f'waypoint {idx} 当前车头点误差: '
                f'dist={last_info["dist"]:.3f} m, '
                f'dyaw={last_info["dyaw"]:.3f} rad ({math.degrees(last_info["dyaw"]):.1f} deg)'
            )

        if reached_photo_zone:
            self.get_logger().info(
                f'waypoint {idx} 已进入拍照区，等待底盘稳定后拍照'
            )

            self.spin_for_duration(self.settle_before_photo_sec)
            self.wait_for_new_image(timeout_sec=self.photo_wait_sec + 1.0)

            save_path = self.record_img(prefix=f'wp_{idx}')
            if save_path is not None:
                self.get_logger().info(
                    f'waypoint {idx} 拍照成功'
                )
                return True
            else:
                self.get_logger().error(
                    f'waypoint {idx} 已进入拍照区，但拍照失败'
                )
                return False

        # 如果没进入拍照区，但导航已经结束，则认为该点失败
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().warn(
                f'waypoint {idx} 导航任务结束，但车头未进入拍照区，判定失败'
            )
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(
                f'waypoint {idx} 导航被取消，且车头未进入拍照区，判定失败'
            )
        else:
            self.get_logger().error(
                f'waypoint {idx} 导航失败，Nav2 result: {result}'
            )

        return False

    # ===========================
    # 巡检主流程
    # ===========================
    def patrol_once(self) -> bool:
        business_points = self.get_target_points()
        if not business_points:
            self.get_logger().error('没有可执行的业务目标点')
            return False

        nav_waypoints = self.build_navigation_targets(business_points)
        if not nav_waypoints or len(nav_waypoints) != len(business_points):
            self.get_logger().error('导航目标构建失败')
            return False

        self.get_logger().info(
            f'开始执行巡检，共 {len(nav_waypoints)} 个 waypoint'
        )

        overall_ok = True
        success_count = 0

        for idx, (business_point, nav_waypoint) in enumerate(
            zip(business_points, nav_waypoints)
        ):
            front_x, front_y, front_yaw = business_point
            business_target_pose = self.get_pose_by_xyyaw(
                front_x, front_y, front_yaw
            )

            ok = self.go_to_waypoint_and_photo(
                idx=idx,
                nav_waypoint=nav_waypoint,
                business_target_pose=business_target_pose
            )

            if ok:
                success_count += 1
            else:
                overall_ok = False
                self.get_logger().warn(
                    f'waypoint {idx} 未完成“到拍照区并拍照”'
                )
                if not self.continue_on_waypoint_failure:
                    self.get_logger().warn(
                        'continue_on_waypoint_failure=False，提前结束本轮巡检'
                    )
                    break

        self.get_logger().info(
            f'本轮巡检结束: 成功拍照 {success_count}/{len(nav_waypoints)}'
        )

        if overall_ok:
            self.get_logger().info('本轮巡检完成，所有巡检点均已进入拍照区并拍照成功')
        else:
            self.get_logger().warn('本轮巡检完成，但存在未成功进入拍照区并拍照的点')

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

