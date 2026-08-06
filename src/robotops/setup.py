from setuptools import setup, find_packages

package_name = 'robotops'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robotops_recorder.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sundasheng',
    maintainer_email='sundasheng@todo.todo',
    description='RobotOps Foundation: SQLite persistence for runtime events',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robotops_recorder_node = robotops.robotops_recorder_node:main',
        ],
    },
)
