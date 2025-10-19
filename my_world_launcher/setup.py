from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_world_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ✅ 安装 launch 文件夹里的所有 .launch.py
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # ✅ 安装 worlds 文件夹里的所有 .world 文件
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='will',
    maintainer_email='will@todo.todo',
    description='Custom world launcher for TurtleBot3 Gazebo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
