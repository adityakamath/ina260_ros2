from glob import glob

from setuptools import find_packages, setup

package_name = 'ina260_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyyaml', 'smbus2'],
    zip_safe=True,
    maintainer='Aditya Kamath (Kamath Robotics)',
    maintainer_email='adityakamath@live.com',
    description='Publishes sensor_msgs/BatteryState from an Adafruit INA260 current/power sensor over I2C, with software coulomb counting for charge/percentage estimation.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_monitor_node = ina260_ros2.battery_monitor_node:main',
            'ina260_calibrate = ina260_ros2.calibrate:main',
        ],
    },
)
