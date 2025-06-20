from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0', # Adjust this to your lidar's serial port  if uart based port /dev/ttyS0
        description='Serial port for the lidar'
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for the lidar'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='rangefinder_link',
        description='Frame ID for the sensor data'
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        frame_id_arg,
        Node(
            package='rangefinder',
            executable='rangefinder_sdk',
            name='rangefinder_data_node',
            output='screen',
            parameters=[
                {'serial_port': LaunchConfiguration('serial_port')},
                {'baud_rate': LaunchConfiguration('baud_rate')},
                {'frame_id': LaunchConfiguration('frame_id')}
            ]
        )
    ])