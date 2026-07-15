from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float_parameter(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    bag_dir = os.path.expanduser(
        f'~/koni_log/sweep_high_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    )
    record_bag = LaunchConfiguration('record_bag')

    bag_record = ExecuteProcess(
        condition=IfCondition(record_bag),
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_dir,
            '-s', 'mcap',
            '/ai1616llpe/voltage',
            '/actuators/valve_voltage',
            '/debug/sweep_voltage_V',
            '/debug/sweep_flow_raw_V',
            '/debug/sweep_is_measuring',
            '/debug/sweep_step_index',
            '/sensors/supply_pressure',
            '/sensors/pam_valve_pressure',
            '/sensors/flow_rate',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('v_start', default_value='5.6'),
        DeclareLaunchArgument('v_end', default_value='10.0'),
        DeclareLaunchArgument('v_step', default_value='0.2'),
        DeclareLaunchArgument('hold_time_s', default_value='1.0'),
        DeclareLaunchArgument('recharge_time_s', default_value='3.0'),
        DeclareLaunchArgument('neutral_voltage', default_value='5.0'),

        # AI board node (sensor acquisition)
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='ai1616llpe_node',
            output='screen',
            parameters=[{
                'update_rate': 100.0,
            }],
        ),

        # AO board node (valve command output)
        Node(
            package='control_box',
            executable='ao1608llpe_test',
            name='ao1608llpe_node',
            output='screen',
        ),

        # Sensor conversion for live monitoring and rosbag recording.
        Node(
            package='py_signal_processing',
            executable='analog_voltage_interpreter_cyl',
            name='sensor_interpreter_node',
            output='screen',
            parameters=[{
                'pam_valve_pressure_index': 1,
                'supply_pressure_index': 6,
                'flowmeter_index': 15,
                'cutoff_hz_pressure': 10.0,
                'cutoff_hz_flow': 10.0,
            }],
        ),

        # High-flow sweep: recharge the tank at neutral between steps.
        Node(
            package='valve_sweep',
            executable='sweep_node',
            name='valve_sweep_node',
            output='screen',
            parameters=[{
                'valve_channel': 1,
                'flowmeter_channel': 15,
                'v_start': _float_parameter('v_start'),
                'v_end': _float_parameter('v_end'),
                'v_step': _float_parameter('v_step'),
                'hold_time_s': _float_parameter('hold_time_s'),
                'recharge_time_s': _float_parameter('recharge_time_s'),
                'neutral_voltage': _float_parameter('neutral_voltage'),
                'control_rate_hz': 100.0,
                'one_way': True,
                'recharge_between_steps': True,
            }],
        ),
       bag_record,
    ])
