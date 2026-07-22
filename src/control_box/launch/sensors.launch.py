from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    flowmeter_full_scale = ParameterValue(
        LaunchConfiguration('flowmeter_full_scale_l_min'), value_type=int)

    return LaunchDescription([
        DeclareLaunchArgument(
            'flowmeter_full_scale_l_min',
            default_value='200',
            description='Flowmeter full scale: 200 or 1600 L/min.',
        ),
        # AIボード
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='ai1616llpe_node',
            output='screen'
        ),
        # AOボード
        Node(
            package='control_box',
            executable='ao1608llpe_test',
            name='ao1608llpe_node',
            output='screen',
        ),
        # カウンタボード
        Node(
            package='control_box',
            executable='cnt3204mtlpe_test',
            name='cnt3204mtlpe_node',
            output='screen'
        ),
        # 信号変換・フィルタノード
        Node(
            package='py_signal_processing',
            executable='analog_voltage_interpreter_cyl',
            name='sensor_interpreter_node',
            output='screen',
            parameters=[{
                # Keep this wiring map aligned with
                # cylinder_exp/launch/phase_pam_sine.launch.py.
                'head_pressure_index':       3,
                'rod_pressure_index':        2,
                'loadcell_plus_index':       4,
                'loadcell_minus_index':      5,
                'pam_pressure_index':        7,
                'pam_valve_pressure_index':  1,
                'cutoff_hz_pressure':        10.0,
                # TOKYO METER APM-L: 3 V = 0 L/min,
                # 1～5 V = -FS～FS L/min (ANR).
                'flowmeter_index':             15,
                'v0_flowmeter':                3.0,
                'flowmeter_full_scale_l_min': flowmeter_full_scale,
                'cutoff_hz_flow':              10.0,
            }],
        ),
    ])
