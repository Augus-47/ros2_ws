from setuptools import find_packages, setup

package_name = 'autopartol_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/config", ['config/partol_config.yaml']),
        ('share/ament_index/resource_index/packages', ['resource/autopartol_robot']),
        ('share/autopartol_robot', ['package.xml']),
        ('share/autopartol_robot/launch', ['launch/bringup_patrol.launch.py']),
        ('share/autopartol_robot/config', ['config/partol_config.yaml', 'config/nav2_params.yaml']),
        ('share/autopartol_robot/rviz', ['rviz/nav2_default_view.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chen',
    maintainer_email='chen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'partol_node=autopartol_robot.partol_node:main',
        ],
    },
)
