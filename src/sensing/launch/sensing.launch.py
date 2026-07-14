from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # AIボードノード
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='ai1616llpe_node',
            output='screen',
        ),

        # センサ解釈ノード
        Node(
            package='py_signal_processing',
            executable='analog_voltage_interpreter_cyl',
            name='sensor_interpreter_node',
            output='screen',
            parameters=[{
                'head_pressure_index':       3,
                'rod_pressure_index':        2,
                'loadcell_plus_index':       4,
                'loadcell_minus_index':      5,
                'pam_pressure_index':        7,
                'pam_valve_pressure_index':  1,
                'supply_pressure_index':     6,
                'cutoff_hz_pressure':        10.0,
                'flowmeter_index':           15,
                'v0_flowmeter':              3.0,
                'slope_l_min_per_v_flowmeter': 100.0,
                'cutoff_hz_flow':            10.0,
            }],
        ),
    ])
