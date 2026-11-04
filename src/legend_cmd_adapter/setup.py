import os
from glob import glob
from setuptools import setup

from setuptools import find_packages, setup

package_name = 'legend_cmd_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='15218303+c305automatic-driving@user.noreply.gitee.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'twist_to_vehiclecmd_node = legend_cmd_adapter.twist_to_vehiclecmd_node:main',
        ],
    },
)
