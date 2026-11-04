from setuptools import setup

package_name = 'traj_compare_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@nvidia-desktop',
    description='Trajectory recording and comparison tools for Nav2 BEV A/B test',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'traj_recorder = traj_compare_tools.traj_recorder:main',
            'traj_compare_pub = traj_compare_tools.traj_compare_pub:main',
        ],
    },
)
