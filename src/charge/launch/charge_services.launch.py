from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'battery_port',
        default_value='/dev/ttyUSB0',
        description='Serial device for BMS',
    )
    baud_arg = DeclareLaunchArgument(
        'battery_baud',
        default_value='9600',
        description='BMS serial baud rate',
    )

    battery_params = {
        'port': ParameterValue(LaunchConfiguration('battery_port'), value_type=str),
        'baudrate': ParameterValue(LaunchConfiguration('battery_baud'), value_type=int),
        'query_hz': 0.5,
        'response_timeout_ms': 800,
        'max_consecutive_failures': 3,
        'battery_topic': '/battery',
        'debug_hex': False,
        'write_trigger_topic': '/battery/serial_write_trigger',
        'reserved_write_hex': '',
    }

    return LaunchDescription([
        port_arg,
        baud_arg,
        Node(
            package='charge',
            executable='charge_services',
            name='charge_services',
            output='screen',
        ),
        Node(
            package='charge',
            executable='battery_status_get',
            name='battery_status_get',
            output='screen',
            parameters=[battery_params],
        ),
    ])
