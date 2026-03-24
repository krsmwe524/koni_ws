from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. AIボードノード (control_box)
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='ai1616llpe_node',
            output='screen'
        ),
        # 2. AOボードノード (control_box)
        Node(
            package='control_box',
            executable='ao1608llpe_test',
            name='ao1608llpe_node',
            output='screen'
        ),
        # 3. カウンタボード(エンコーダ)ノード (control_box)
        Node(
            package='control_box',
            executable='cnt3204mtlpe_test',
            name='cnt3204mtlpe_node',
            output='screen'
        ),
        # 4. 信号変換・フィルタノード
        Node(
            package='py_signal_processing',
            executable='analog_voltage_interpreter_cyl',
            name='sensor_interpreter_node',
            output='screen',
            parameters=[{
                'head_pressure_index': 0, 
                'rod_pressure_index': 1,

            }]
        ),

        Node(
            package='cylinder_exp',
            executable='pos_controller',
            name='cylinder_position_controller_node',
            output='screen',
            parameters=[{
                'ch_head': 0,
                'ch_rod': 1,
                'sine_amplitude_m': 0.004,  # 振幅 (40mm)
                'sine_freq_hz': 0.5,        # 周波数 (0.5Hz)
                'center_position_m': 0.005, # 中心位置 (例: 50mm)

                'base_pressure_kpa': 150.0, # ベース圧力
                'supply_pressure_kpa': 600.0, # 供給圧力の上限

                # 位置ループ（外側）PIDゲイン
                'pos_kp': 2000.0,
                'pos_ki': 0.0, 
                'pos_kd': 0.0,
                'pos_td': 0.05,   # 微分フィルタ時定数

                # 圧力ループ（内側）PIDゲイン
                'pres_kp': 0.01,
                'pres_ki': 0.01,
                'pres_kd': 0.0,
                'pres_td': 0.01,
            }]
        ),
    ])