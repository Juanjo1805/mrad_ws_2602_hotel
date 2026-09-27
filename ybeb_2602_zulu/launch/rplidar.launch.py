"""Start the existing real RPLIDAR using its unchanged persistent USB path."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Keep the original port, baud rate and laser_frame calibration."""
    return LaunchDescription([
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': ('/dev/serial/by-id/usb-Silicon_Labs_CP2102_'
                                'USB_to_UART_Bridge_Controller_0001-if00-port0'),
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])
