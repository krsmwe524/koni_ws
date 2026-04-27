from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
                'head_pressure_index':  1,
                'rod_pressure_index':   0,
                'loadcell_plus_index':  2,
                'loadcell_minus_index': 3,
                'pam_pressure_index':   7,
                'cutoff_hz_pressure':   10.0,
            }],
        ),
    ])