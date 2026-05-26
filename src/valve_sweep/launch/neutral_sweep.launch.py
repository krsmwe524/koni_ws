from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bag_dir = os.path.expanduser(
        f'~/koni_log/neutral_sweep_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    )

    initial_voltage = LaunchConfiguration('initial_voltage')
    target_voltage = LaunchConfiguration('target_voltage')
    # valve_channel = LaunchConfiguration('valve_channel')
    monitor_ai_channel = LaunchConfiguration('monitor_ai_channel')
    sample_rate_hz = LaunchConfiguration('sample_rate_hz')

    initial_voltage_param = ParameterValue(initial_voltage, value_type=float)
    target_voltage_param = ParameterValue(target_voltage, value_type=float)
    # valve_channel_param = ParameterValue(valve_channel, value_type=int)
    monitor_ai_channel_param = ParameterValue(monitor_ai_channel, value_type=int)
    sample_rate_hz_param = ParameterValue(sample_rate_hz, value_type=float)

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_dir,
            '-s', 'mcap',
            '/ai1616llpe/voltage',
            '/debug/neutral_sweep_ai_ch6_raw_V',
            '/sensors/pam_valve_pressure',
            '/actuators/valve_voltage',
            '/debug/neutral_sweep_valve_voltage_V',
        ],
        output='screen',
    )

    ai_node = Node(
        package='control_box',
        executable='ai1616llpe_test',
        name='ai1616llpe_node',
        output='screen',
        parameters=[{
            'update_rate': sample_rate_hz_param,
        }],
    )

    ao_node = Node(
        package='control_box',
        executable='ao1608llpe_test',
        name='ao1608llpe_node',
        output='screen',
    )

    sensor_interpreter_node = Node(
        package='py_signal_processing',
        executable='analog_voltage_interpreter_cyl',
        name='sensor_interpreter_node',
        output='screen',
        parameters=[{
            'pam_valve_pressure_index': monitor_ai_channel_param,
            'cutoff_hz_pressure': 10.0,
        }],
    )

    neutral_sweep_node = Node(
        package='valve_sweep',
        executable='neutral_sweep_node',
        name='neutral_sweep_node',
        output='screen',
        parameters=[{
            'valve_channel': 1,
            'monitor_ai_channel': monitor_ai_channel_param,
            'initial_voltage': initial_voltage_param,
            'target_voltage': target_voltage_param,
            'initial_duration_s': 2.0,
            'target_duration_s': 130.0,
            'sample_rate_hz': sample_rate_hz_param,
            'neutral_default': 5.0,
        }],
    )

    shutdown_after_sweep = RegisterEventHandler(
        OnProcessExit(
            target_action=neutral_sweep_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='neutral sweep completed')),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('initial_voltage', default_value='5.6'),
        DeclareLaunchArgument('target_voltage', default_value='5.8'),
        # DeclareLaunchArgument('valve_channel', default_value='1'),
        DeclareLaunchArgument('monitor_ai_channel', default_value='6'),
        DeclareLaunchArgument('sample_rate_hz', default_value='100.0'),
        # bag_record,
        ai_node,
        ao_node,
        sensor_interpreter_node,
        neutral_sweep_node,
        shutdown_after_sweep,
    ])
