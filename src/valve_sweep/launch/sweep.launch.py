from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from datetime import datetime
import os


def generate_launch_description():
    bag_dir = os.path.expanduser(
        f'~/koni_log/sweep_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
    record_bag = LaunchConfiguration('record_bag')
    flowmeter_full_scale = ParameterValue(
        LaunchConfiguration('flowmeter_full_scale_l_min'), value_type=int)

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
            '/sensors/flowmeter_full_scale',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument(
            'flowmeter_full_scale_l_min',
            default_value='200',
            description='Flowmeter full scale: 200 or 1600 L/min.',
        ),

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

        # センサ物理量変換ノード
        Node(
            package='py_signal_processing',
            executable='analog_voltage_interpreter_cyl',
            name='sensor_interpreter_node',
            output='screen',
            parameters=[{
                'pam_valve_pressure_index': 1,
                'supply_pressure_index': 6,
                'flowmeter_index': 15,
                'flowmeter_full_scale_l_min': flowmeter_full_scale,
                'cutoff_hz_pressure': 10.0,
                'cutoff_hz_flow': 10.0,
            }],
        ),

        # スイープノード
        Node(
            package='valve_sweep',
            executable='sweep_node',
            name='valve_sweep_node',
            output='screen',
            parameters=[{
                'valve_channel':  1, #1と3 1はヘッド 3はロッド
                'flowmeter_channel': 15, #流量計のチャンネル
                'v_start':          4.6,
                'v_end':            6.6,
                'v_step':           0.2, #0.05
                'hold_time_s':      1.0,
                'control_rate_hz':  100.0,
                'one_way':          True, # True: 片道, False: 往復
            }],
        ),
        #bag_record,
    ])
