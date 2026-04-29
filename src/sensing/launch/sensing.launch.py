from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from datetime import datetime
import os


def generate_launch_description():
    bag_dir = os.path.expanduser(
        f'~/koni_log/MVC_{datetime.now().strftime("%Y%m%d_%H%M%S")}')

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_dir,
            '-s', 'mcap',
            '-e', '/sensors/.*',
        ],
        output='screen',
    )

    return LaunchDescription([
        bag_record,

        # AIボードノード
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='ai1616llpe_node',
            output='screen',
        ),

        # センサ解釈ノード
        Node(
            package='sensing',
            executable='analog_interpreter',
            name='analog_interpreter_node',
            output='screen',
            parameters=[{
                'ai_topic':              '/ai1616llpe/voltage',
                # 圧力センサ (ch7)
                'pressure_index':        7,
                'v0_pressure':           1.0,
                'slope_kPa_per_V':       250.0,
                # ロードセル (ch・ゲインは実機に合わせて変更)
                'loadcell_plus_index':   2,
                'loadcell_minus_index':  3,
                'v0_loadcell':           0.0,
                'kg_per_V_loadcell':     7.9186,
                'gravity_acceleration':  9.80665,
                # ワイヤ式長さセンサ (ch6)
                'wire_index':            6,
                'pos_fullscale_v':       5.0,
                'pos_fullscale_mm':      1000.0,
                # EMG (ch1: 三角筋, ch2: 上腕三頭筋)
                'emg_deltoid_index':     0,
                'emg_triceps_index':     1,
                'emg_rms_window_ms':     500,
            }],
        ),
    ])
