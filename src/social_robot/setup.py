# social_robot/setup.py
from setuptools import setup, find_packages
from glob import glob

package_name = 'social_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),   # ★ 自动找到 social_robot/ 目录
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='Social robot face follow module',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_follow_node = social_robot.face_follow_node:main',
            'gesture_player_node = social_robot.gesture_player_node:main',
            'env_scan_node = social_robot.env_scan_node:main',
            'static_env_report_node = social_robot.static_env_report_node:main',
            'world_model_node = social_robot.world_model_node:main',
        ],
    },
)

