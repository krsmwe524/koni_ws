from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from datetime import datetime
import os


# PAM pressure controller feedback source.
# Use '/sensors/pam_valve_pressure' for valve-side control,
# or '/sensors/pam_pressure' for PAM-side control.
PAM_CONTROL_PRESSURE_TOPIC = '/sensors/pam_pressure'


def generate_launch_description():
    """Smooth random cylinder-position trajectory for MC-style experiments.

    Channel and sensor assignments follow cylinder_mseq_mc.launch.py.
    The cylinder is position-controlled with smooth random waypoints instead of
    open-loop M-sequence voltage steps.
    """
    bag_dir = os.path.expanduser(
        f'~/koni_log/RW_MC_{datetime.now().strftime("%Y%m%d_%H%M%S")}')

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_dir,
            '-s', 'mcap',
            # アクチュエータ
            '/actuators/cylinder_valves',
            '/actuators/pam_valve',
            '/actuators/valve_voltage',
            # センサ
            '/sensors/cylinder_position',
            '/sensors/head_pressure',
            '/sensors/rod_pressure',
            '/sensors/loadcell_force',
            '/sensors/pam_pressure',
            '/sensors/pam_valve_pressure',
            '/sensors/supply_pressure',
            # ランダム目標位置制御のデバッグ
            '/debug/random_waypoint_value',
            '/debug/random_waypoint_index',
            '/debug/current_random_amplitude_m',
            '/debug/target_position_m',
            '/debug/current_rel_position_m',
            '/debug/position_error_m',
            '/debug/target_force_N',
            '/debug/pid_force_N',
            '/debug/target_pressure_head_kPa',
            '/debug/target_pressure_rod_kPa',
            '/debug/current_pressure_head_kPa',
            '/debug/current_pressure_rod_kPa',
            '/debug/valve_delta_head_V',
            '/debug/valve_delta_rod_V',
            # PAM 定圧制御のデバッグ
            '/debug/pam_target_pressure_kPa',
            '/debug/pam_control_pressure_kPa',
            '/debug/pam_control_pressure_error_kPa',
            '/debug/pam_pressure_error_kPa',
            '/debug/pam_pressure_error_derivative_kPa_s',
            '/debug/pam_valve_output_V',
        ],
        output='screen',
    )

    return LaunchDescription([
        bag_record,
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='ai1616llpe_node',
            output='screen',
        ),
        Node(
            package='control_box',
            executable='ao1608llpe_test',
            name='ao1608llpe_node',
            output='screen',
        ),
        Node(
            package='control_box',
            executable='cnt3204mtlpe_test',
            name='cnt3204mtlpe_node',
            output='screen',
        ),
        Node(
            package='py_signal_processing',
            executable='analog_voltage_interpreter_cyl',
            name='sensor_interpreter_node',
            output='screen',
            parameters=[{
                'head_pressure_index':  1,
                'rod_pressure_index':   0,
                'loadcell_plus_index':  2,
                'loadcell_minus_index': 3,
                'pam_pressure_index':   7,
                'pam_valve_pressure_index': 6,
                'cutoff_hz_pressure':   10.0,
            }],
        ),
        Node(
            package='cylinder_exp',
            executable='random_waypoint_pos_controller',
            name='random_waypoint_position_controller_node',
            output='screen',
            parameters=[{
                # cylinder_mseq_mc.launch.py と同じAOチャンネル
                'ch_head': 3,
                'ch_rod':  2,

                # ループ周期
                'outer_rate_hz': 500.0,
                'inner_rate_hz': 1000.0,

                # 滑らかなランダム目標位置
                # x_ref_rel は [0, 2 * random_amplitude_m] の範囲。
                'random_amplitude_m': 0.020,
                'random_amp_ramp_rate_m_s': 0.0005,
                'waypoint_period_s': 0.5,
                'waypoint_seed': 5,

                # 圧力
                'base_pressure_kpa':   250.0,
                'supply_pressure_kpa': 600.0,

                # 位置ループ PID（cylinder_control.launch.py を基準）
                'pos_kp': 1500.0,
                'pos_ki': 30.0,
                'pos_kd': 0.0,
                'pos_td': 1.0,
                'pos_output_limit': 1000.0,

                # 圧力ループ PID
                'pres_kp': 0.016,
                'pres_ki': 0.002,
                'pres_kd': 0.0,
                'pres_td': 0.01,
                'pres_output_limit': 4.9,

                # 起動待機: cylinder_mseq_mc.launch.py と同様に、
                # この電圧を startup_wait_s 秒だけ出し、その最後の位置を x_0 にする。
                'startup_wait_s':         8.0,
                'startup_head_voltage_v': 0.0,
                'startup_rod_voltage_v':  8.0,

                # ロードセル補償
                'use_loadcell_compensation': False,
                'loadcell_ff_gain': 1.0,
                'loadcell_timeout_s': 3.2,
            }],
        ),
        Node(
            package='cylinder_exp',
            executable='pam_const_pressure_controller',
            name='pam_const_pressure_controller_node',
            output='screen',
            parameters=[{
                'target_pressure_kpa': 250.0,
                'kp':                  0.048,
                'ki':                  0.00,
                'kd':                  0.0023,
                'td':                  0.08,
                'derivative_enable_delay_s': 10.0,
                'output_limit':        4.9,
                'valve_channel':       1,
                'control_rate_hz':     1000.0,
                'control_topic':       PAM_CONTROL_PRESSURE_TOPIC,
                'valve_topic':         '/actuators/pam_valve',
            }],
        ),
        Node(
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
        ),
    ])
