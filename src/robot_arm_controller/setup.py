from setuptools import find_packages, setup

package_name = "robot_arm_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dzubokko",
    maintainer_email="dzubokko@example.com",
    description="GUI for controlling a custom robot manipulator in ROS 2 and Gazebo.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot-controller = robot_arm_controller.app:main",
        ],
    },
)