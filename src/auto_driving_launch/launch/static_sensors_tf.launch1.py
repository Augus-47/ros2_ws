from launch import LaunchDescription
from launch_ros.actions import Node
import math


def quat_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """
    Convert RPY in degrees (roll, pitch, yaw) to quaternion (x, y, z, w).
    Assumes aerospace/ROS convention: roll around X, pitch around Y, yaw around Z.
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr = math.cos(r * 0.5)
    sr = math.sin(r * 0.5)
    cp = math.cos(p * 0.5)
    sp = math.sin(p * 0.5)
    cy = math.cos(y * 0.5)
    sy = math.sin(y * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


def static_tf_node(name: str,
                   parent: str, child: str,
                   x: float, y: float, z: float,
                   qx: float, qy: float, qz: float, qw: float):
    # tf2_ros/static_transform_publisher quaternion args:
    # x y z qx qy qz qw frame_id child_frame_id
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        output='screen',
        arguments=[
            f'{x:.12f}', f'{y:.12f}', f'{z:.12f}',
            f'{qx:.12f}', f'{qy:.12f}', f'{qz:.12f}', f'{qw:.12f}',
            parent, child
        ]
    )


def generate_launch_description():
    # -------------------------
    # map -> odom  (identity)
    # -------------------------
    odom_to_base_link = static_tf_node(
        name='tf_odom_to_baselink',
        parent='odom',
        child='base_link',
        x=0.0, y=0.0, z=0.0,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0
    )

    # -------------------------
    # base_link -> rslidar  (Euler=0,0,0 so quaternion identity)
    # -------------------------
    base_to_lidar = static_tf_node(
        name='tf_base_to_rslidar',
        parent='base_link',
        child='rslidar',
        x=0.42, y=0.0, z=1.0,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0
    )

    # -------------------------
    # rslidar -> imu_link  (Euler degrees -> quaternion)
    # rpy(deg) = [179.762270, 0.070193, -4.322902]
    # -------------------------
    # qx, qy, qz, qw = quat_from_rpy_deg(179.762270, 0.070193, -4.322902)
    qx, qy, qz, qw = quat_from_rpy_deg(0.0, 0.070193, 0.0)
    lidar_to_imu = static_tf_node(
        name='tf_rslidar_to_imu',
        parent='rslidar',
        child='imu_link',
        x=0.36, y=-0.14, z=-0.55,
        qx=qx, qy=qy, qz=qz, qw=qw
    )

    # -------------------------
    # rslidar -> camera_front (t + quaternion)
    # T_lidar_camera = [tx, ty, tz, qx, qy, qz, qw]
    # -------------------------
    cam_front = static_tf_node(
        name='tf_rslidar_to_camera_front',
        parent='rslidar',
        child='camera_front',
        x=0.16550511772856422,
        y=-0.0003163653217201284,
        z=-0.041388052276160554,
        qx=-0.5042146903695429,
        qy=0.4975212735222683,
        qz=-0.5002103555111131,
        qw=0.49802583130573863
    )

    cam_right = static_tf_node(
        name='tf_rslidar_to_camera_right',
        parent='rslidar',
        child='camera_right',
        x=0.0031314587226820016,
        y=-0.23814783903240186,
        z=-0.049170490956282044,
        qx=0.011944211869822285,
        qy=0.7193899174882676,
        qz=-0.6943875953975415,
        qw=-0.012702353209248417
    )

    cam_rear = static_tf_node(
        name='tf_rslidar_to_camera_rear',
        parent='rslidar',
        child='camera_rear',
        x=-0.17550511772856422,
        y=-0.0003163653217201284,
        z=-0.041388052276160554,
        qx=-0.49969541,
        qy=-0.51744975,
        qz=0.48255025,
        qw=0.49969541
    )

    cam_left = static_tf_node(
        name='tf_rslidar_to_camera_left',
        parent='rslidar',
        child='camera_left',
        x=0.0031314587226820016,
        y=0.23814783903240186,
        z=-0.049170490956282044,
        qx=-0.69465837,
        qy=0.0,
        qz=0.0,
        qw=0.71933980
    )

    return LaunchDescription([
        odom_to_base_link,
        base_to_lidar,
        lidar_to_imu,
        cam_front, cam_right, cam_rear, cam_left
    ])

