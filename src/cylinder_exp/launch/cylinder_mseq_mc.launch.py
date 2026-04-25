from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from datetime import datetime
import os


def generate_launch_description():
    """
    M系列を用いた MC (Memory Capacity) 評価実験用 launch。

    シリンダ側: 2つのサーボバルブ (ch1=head, ch3=rod) を M系列で差動駆動 (開ループ)
    PAM 側   : pam_const_pressure_controller で圧力一定制御 (ch6)
    """
    bag_dir = os.path.expanduser(
        f'~/koni_log/mc_{datetime.now().strftime("%Y%m%d_%H%M%S")}')

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_dir,
            '-s', 'mcap',
            # アクチュエータ
            '/actuators/cylinder_valves',
            '/actuators/pam_valve',
            '/actuators/valve_voltage',
            # センサ (1kHz 物理量)
            '/sensors/cylinder_position',
            '/sensors/head_pressure',
            '/sensors/rod_pressure',
            '/sensors/loadcell_force',
            '/sensors/pam_pressure',
            # M系列ドライバのデバッグ
            '/debug/mseq_value',
            '/debug/mseq_cycle_index',
            '/debug/mseq_amplitude_v',
            '/debug/cylinder_v_head',
            '/debug/cylinder_v_rod',
            # PAM 定圧制御のデバッグ
            '/debug/pam_target_pressure_kPa',
            '/debug/pam_pressure_error_kPa',
            '/debug/pam_valve_output_V',
        ],
        output='screen',
    )

    return LaunchDescription([
        bag_record,

        # AI ボードノード
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='ai1616llpe_node',
            output='screen',
        ),

        # AO ボードノード
        Node(
            package='control_box',
            executable='ao1608llpe_test',
            name='ao1608llpe_node',
            output='screen',
        ),

        # カウンタボードノード
        Node(
            package='control_box',
            executable='cnt3204mtlpe_test',
            name='cnt3204mtlpe_node',
            output='screen',
        ),

        # 信号変換・フィルタノード
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
                'cutoff_hz_pressure':   10.0,
            }],
        ),

        # シリンダ M系列差動駆動ノード (開ループ)
        Node(
            package='cylinder_exp',
            executable='cylinder_mseq_driver',
            name='cylinder_mseq_driver_node',
            output='screen',
            parameters=[{
                # AOボードとの接続チャンネル
                'ch_head': 1,
                'ch_rod':  3,

                # M系列差動駆動
                'amplitude_v':         0.5,    # A [V] (1V前後を目安)
                'mseq_order':          12,     # n=12 → 系列長 4095
                'mseq_clock_period_s': 0.010,  # T_c = 10ms (1kHzサンプリングで10サンプル毎に切替)
                'mseq_seed':           1,      # 訓練用シード (評価用は別 launch で 2 などに)

                # タイミング
                'update_rate_hz':      1000.0,
                'startup_wait_s':      3.0,    # PAM圧力安定化のための待機
                'amp_ramp_duration_s': 1.0,    # 振幅 0 → A をかけるランプ時間

                # 系列の終了挙動
                'loop_sequence':       True,   # 1周完了後ループするか
                'max_cycles':          0,      # 0=無制限 / 1サイクル=4095*0.01=40.95s
            }],
        ),

        # PAM 定圧制御ノード
        Node(
            package='cylinder_exp',
            executable='pam_const_pressure_controller',
            name='pam_const_pressure_controller_node',
            output='screen',
            parameters=[{
                'target_pressure_kpa': 100.0,
                'kp':                  0.0,  # 0.02
                'ki':                  0.0,  # 0.005
                'output_limit':        4.9,
                'valve_channel':       6,
                'control_rate_hz':     500.0,
                'pressure_topic':      '/sensors/pam_pressure',
                'valve_topic':         '/actuators/pam_valve',
            }],
        ),

        # バルブ指令統合ノード
        Node(
            package='cylinder_exp',
            executable='mixer_node',
            name='valve_mixer_node',
            output='screen',
            parameters=[{
                'output_rate_hz': 1000.0,
                'input_topics': [
                    '/actuators/cylinder_valves',
                    '/actuators/pam_valve',
                ],
            }],
        ),
    ])
