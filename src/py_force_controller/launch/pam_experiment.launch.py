import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    log_dir = "/home/kklab/koni_ws/src/py_force_controller/prbs_log"
    
    return LaunchDescription([
    # AI
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='AI_board',
            output='screen',
        ),
    # AO
        Node(
            package='control_box',
            executable='ao1608llpe_test',
            name='AO_board',
            output='screen',
        ),
        Node(
            package='py_signal_processing',
            executable='analog_voltage_interpreter',
            name='analog_voltage_interpreter',
            parameters=[{
                'ai_channels': 16,
                #'index_map': [1, 2, 5, 4, 7, 6],
                'index_map': [1, 2, 5, 0, 7, 6],
                'load_v_zero_N': 0.0,
                'load_gain_N_per_v': 116.6196
            }]
        ),

# PRBS生成ノード
#         Node(
#             package='py_force_controller',
#             executable='prbs_generator',
#             name='prbs_generator',
#             parameters=[{
#     'start_neutral_sec': 3.0,
#     'washout_sec': 3.0,
#     'duration_sec': 60.0,
#     'end_neutral_sec': 3.0,

#     # PRBSホールド（固定0.3s）
#     'hold_dt_mode': 'fixed',
#     'hold_dt_fixed': 0.3,

#     # 中立電圧
#     'neutral_v': 5.0,

#     # 完全ランダムで使う10レベル
#     'levels': [3.8, 4.2, 4.6, 5.4, 5.8, 6.2, 6.6, 7.0, 7.4],

#     # 4回以上同じ側が続かない（=最大3連続）
#     'max_same_side_run': 3,

#     # 左右同一出力（いまの運用）
#     'same_lr': True,

#     # 乱数シード（左右別seedは廃止して、seed 1本にしてます）
#     'seed': 20,
# }]
#         ),
Node(
    package='py_force_controller',
    executable='prbs_generator',
    name='prbs_generator',
    parameters=[{
        'start_neutral_sec': 3.0,
        'washout_sec': 3.0,
        'duration_sec': 60.0,
        'end_neutral_sec': 3.0,

        'hold_dt_mode': 'fixed',
        'hold_dt_fixed': 0.3,

        'neutral_v': 5.0,

        'levels': [3.8, 4.2, 4.6, 5.4, 5.8, 6.2, 6.6, 7.0, 7.4],

        # 連続禁止ルールは使わない
        'max_same_side_run': 0,

        'same_lr': True,
        'seed': 20,

        # --- p2 guard ---
        'enable_p2_guard': True,
        'p2_topic': '/sensors/R_pres_kpa',   # ← PAM2側の圧に合わせて
        'p2_min_kpa': 80.0,
        'p2_max_kpa': 400.0,
    }]
),

        #log
        Node( 
            package='py_force_controller',
            executable='experiment_logger',
            name='experiment_logger',
            parameters=[{
                'log_dir': "/home/kklab/koni_ws/src/py_force_controller/prbs_log"
            }]
        )
    ])
