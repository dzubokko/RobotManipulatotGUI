from setuptools import setup, find_packages

package_name = 'robot_arm_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'PyQt6',
        'rclpy',
        'geometry-msgs',
        'sensor-msgs',
        'std-msgs',
        'trajectory-msgs',
    ],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='GUI для управления UR5 в Gazebo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot-controller = robot_arm_controller.main:main',
        ],
    },
)
