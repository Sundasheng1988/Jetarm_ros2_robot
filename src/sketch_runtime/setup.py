from setuptools import setup, find_packages

package_name = 'sketch_runtime'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/runtime_test.launch.py',
            'launch/ground_runtime_bringup.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sundasheng',
    maintainer_email='sundasheng@todo.todo',
    description='Robot Runtime v0.1: TaskContext, TargetObject, Skill system abstraction layer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'runtime_test_node = sketch_runtime.runtime_test_node:main',
            'real_grounded_runtime_node = sketch_runtime.real_grounded_runtime_node:main',
        ],
    },
)
