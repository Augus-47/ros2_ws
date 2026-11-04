from setuptools import find_packages, setup

package_name = 'camera_driver'

from setuptools import setup, find_packages

package_name = 'camera_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # 新增：强制指定要安装的脚本，确保编译时复制到可执行路径
    scripts=['camera_pub_node.py'],  # 这里填脚本的实际文件名（根目录下）
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # 补充依赖：确保包含ROS 2和相机驱动可能用到的库
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'opencv-python'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='15218303+c305automatic-driving@user.noreply.gitee.com',
    description='Camera driver package for ROS 2',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    # 保留entry_points，确保ros2 run能识别节点（与scripts参数互补）
    entry_points={
        'console_scripts': [
            'camera_pub_node = camera_pub_node:main',  # 修正：直接用脚本名（无需包前缀）
        ],
    },
)

