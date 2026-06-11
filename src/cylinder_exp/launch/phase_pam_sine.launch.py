from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bag_dir = os.path.expanduser(
        f'~/koni_log/phase_pam_sine_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    )

    record_bag = LaunchConfiguration('record_bag')
    startup_stabilize_s = LaunchConfiguration('startup_stabilize_s')
    sine_freq_hz = LaunchConfiguration('sine_freq_hz')
    sine_amplitude_m = LaunchConfiguration('sine_amplitude_m')
    pam_valve_channel = LaunchConfiguration('pam_valve_channel')
    pam_control_topic = LaunchConfiguration('pam_control_topic')

    startup_stabilize_param = ParameterValue(
        startup_stabilize_s, value_type=float
    )
    sine_freq_param = ParameterValue(sine_freq_hz, value_type=float)
    sine_amplitude_param = ParameterValue(sine_amplitude_m, value_type=float)
    pam_valve_channel_param = ParameterValue(
        pam_valve_channel, value_type=int
    )

    bag_record = ExecuteProcess(
        condition=IfCondition(record_bag),
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_dir,
            '-s', 'mcap',
            '/actuators/valve_voltage',
            '/sensors/cylinder_position',
            '/sensors/head_pressure',
            '/sensors/rod_pressure',
            '/sensors/pam_pressure',
            '/sensors/supply_pressure',
            '/sensors/pam_valve_pressure',
            '/sensors/loadcell_force',
            '/debug/target_position_m',
            '/debug/sine_phase_rad',
            '/debug/pam_target_pressure_kPa',
            '/debug/pam_control_pressure_kPa',
            '/debug/pam_valve_output_V',
        ],
        output='screen',
    )

    ai_node = Node(
        package='control_box',
        executable='ai1616llpe_test',
        name='ai1616llpe_node',
        output='screen',
    )

    ao_node = Node(
        package='control_box',
        executable='ao1608llpe_test',
        name='ao1608llpe_node',
        output='screen',
    )

    cnt_node = Node(
        package='control_box',
        executable='cnt3204mtlpe_test',
        name='cnt3204mtlpe_node',
        output='screen',
    )

    sensor_interpreter_node = Node(
        package='py_signal_processing',
        executable='analog_voltage_interpreter_cyl',
        name='sensor_interpreter_node',
        output='screen',
        parameters=[{
            'head_pressure_index': 0,
            'rod_pressure_index': 1,
            'loadcell_plus_index': 2,
            'loadcell_minus_index': 3,
            'pam_pressure_index': 7,
            'pam_valve_pressure_index': 6,
            'cutoff_hz_pressure': 10.0,
        }],
    )

    controller_node = Node(
        package='cylinder_exp',
        executable='phase_pam_sine_pos_controller',
        name='phase_pam_sine_position_controller_node',
        output='screen',
        parameters=[{
            # AO channels
            'ch_head': 3,
            'ch_rod': 2,
            'startup_head_voltage_v': 0.0,
            'startup_rod_voltage_v': 8.0,
            'startup_stabilize_s': startup_stabilize_param,

            # Sine position target. phase=0 is the valley at x_0.
            'sine_freq_hz': sine_freq_param,
            'sine_amplitude_m': sine_amplitude_param,
            'sine_amp_ramp_rate_m_s': 0.010,

            # Cylinder control.
            'outer_rate_hz': 500.0,
            'inner_rate_hz': 1000.0,
            'base_pressure_kpa': 250.0,
            'supply_pressure_kpa': 600.0,
            'pos_kp': 3500.0,
            'pos_ki': 0.0,
            'pos_kd': 0.0,
            'pos_td': 1.0,
            'pos_output_limit': 1000.0,
            'pres_kp': 0.024,
            'pres_ki': 0.05,
            'pres_kd': 0.0,
            'pres_td': 0.01,
            'pres_output_limit': 4.9,
            # PAM control
            'pam_valve_channel': pam_valve_channel_param,
            'pam_control_topic': pam_control_topic,
            'pam_control_rate_hz': 1000.0,
            'pam_low_pressure_kpa': 100.0,
            'pam_high_pressure_kpa': 100.0, #350.0
            'pam_high_start_ratio': 0.10, #0.4
            'pam_high_end_ratio': 0.20, #0.5
            'pam_warmup_cycles': 10,
            'pam_hold_kp': 0.03,
            'pam_hold_ki': 0.01,
            'pam_step_kp': 0.03,
            'pam_step_ki': 0.0,
            'pam_output_limit': 4.9,
        }],
    )

    mixer_node = Node(
        package='cylinder_exp',
        executable='mixer_node',
        name='valve_mixer_node',
        output='screen',
        parameters=[{
            'output_rate_hz': 1000.0,
            'initial_voltages': [
                5.0, 5.0, 8.0, 0.0,
                5.0, 5.0, 5.0, 5.0,
            ],
            'input_topics': [
                '/actuators/cylinder_valves',
                '/actuators/pam_valve',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('startup_stabilize_s', default_value='5.0'),
        DeclareLaunchArgument('sine_freq_hz', default_value='1.0'),
        DeclareLaunchArgument('sine_amplitude_m', default_value='0.010'),
        DeclareLaunchArgument('pam_valve_channel', default_value='1'),
        DeclareLaunchArgument(
            'pam_control_topic',
            default_value='/sensors/pam_pressure',
        ),
        bag_record,
        ai_node,
        ao_node,
        cnt_node,
        sensor_interpreter_node,
        controller_node,
        mixer_node,
    ])
