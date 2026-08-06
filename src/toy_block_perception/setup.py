from setuptools import find_packages, setup

package_name = "toy_block_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/toy_block_color_detector.launch.py"]),
        ("share/" + package_name + "/config", ["config/toy_block_color_detector.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sundasheng",
    maintainer_email="sundasheng@example.com",
    description="ROS 2 live toy-block RGB detector and five-color classifier.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "toy_block_color_detector = toy_block_perception.color_detector_node:main",
        ],
    },
)
