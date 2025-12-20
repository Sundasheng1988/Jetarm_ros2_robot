import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 自动包含所有 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 如果你之后有 config/ 也可加： (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sundasheng',
    maintainer_email='sundasheng@todo.todo',
    description='app for tag pose comparison',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'working_calibration_node = app.working_calibration_node:main',
            'working_calibration_pose_node = app.working_calibration_pose_node:main',
            'get_robot_pose_client = app.get_robot_pose_client:main',
            'calibration_node = app.calibration_node:main',
            'test_calculation = app.test_calculation: main',
            'object_detection = app.object_detection_node: main',
            'roi_color_detector_node = app.roi_color_detector_node:main',
        ]
    },
)

