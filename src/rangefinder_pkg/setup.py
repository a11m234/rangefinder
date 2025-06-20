from setuptools import find_packages, setup

package_name = 'rangefinder_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rangefinder_launch.py']), # Add this line for the launch file
    ],
    install_requires=['setuptools', 'pyserial', 'rclpy', 'sensor_msgs'], # Added sensor_msgs and pyserial
    zip_safe=True,
    maintainer='akshai',
    maintainer_email='akshai',
    description='ROS 2 driver for 1D laser rangefinder EY09A',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rangefinder_sdk = rangefinder_pkg.rangefinder_sdk:main', # This defines your executable
        ],
    },
)