from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from datetime import datetime
import os


def generate_launch_description():
    bag_dir = os.path.expanduser(
        f'~/koni_log/Neutral_{datetime.now().strftime("%Y%m%d_%H%M%S")}_3')
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_dir,
            '-s', 'mcap',
            # '/actuators/valve_voltage',
            '/ai1616llpe/voltage',
            '/debug/sweep_voltage_V',
            #'/debug/sweep_flow_raw_V',
        ],
        output='screen',
    )

    return LaunchDescription([
      # bag_record,
        # AI ボードノード (センサ読み取り)
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='ai1616llpe_node',
            output='screen',
            parameters=[{
                'update_rate': 100.0,  # AIボードのサンプリングレート [Hz]
            }],
        ),

        # AO ボードノード (DAC出力)
        Node(
            package='control_box',
            executable='ao1608llpe_test',
            name='ao1608llpe_node',
            output='screen',
            
        ),

        # スイープノード
        Node(
            package='valve_sweep',
            executable='sweep_node',
            name='valve_sweep_node',
            output='screen',
            parameters=[{
                'valve_channel':  1, #1と3 1はヘッド 3はロッド
                'flowmeter_channel': 1, #流量計のチャンネル
                'v_start':          4.6,
                'v_end':            4.6,
                'v_step':           0.05, #0.05
                'hold_time_s':      12.0,
                'control_rate_hz':  100.0,
                'one_way':          True, # True: 片道, False: 往復
            }],
        ),
    ])
