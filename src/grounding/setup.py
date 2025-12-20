from setuptools import setup
package_name = 'grounding'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', ['config/grounding_params.yaml']),
        (f'share/{package_name}/launch', ['launch/ground_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sundasheng',
    maintainer_email='sundasheng@todo.todo',
    description='Grounding node: map parsed command to concrete object_id and poses.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'grounding_node = grounding.grounding_node:main',
            'wm_dummy_pub = grounding.wm_dummy_pub:main',
            'wm_from_tf = grounding.wm_from_tf:main',
        ],
    },
)

