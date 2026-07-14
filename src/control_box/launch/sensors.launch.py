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
                # Keep this wiring map aligned with
                # cylinder_exp/launch/phase_pam_sine.launch.py.
                'head_pressure_index':       3,
                'rod_pressure_index':        2,
                'loadcell_plus_index':       4,
                'loadcell_minus_index':      5,
                'pam_pressure_index':        7,
                'pam_valve_pressure_index':  1,
                'cutoff_hz_pressure':        10.0,
                # TOKYO METER APM-L-200D: 3 V = 0 L/min,
                # 1～5 V = -200～200 L/min (ANR).
                'flowmeter_index':                 15,
                'v0_flowmeter':                    3.0,
                'slope_l_min_per_v_flowmeter':   100.0,
                'cutoff_hz_flow':                  10.0,
            }],
        ),
    ])
