# variable_force.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os, datetime
from launch.actions import ExecuteProcess

# ---- ログ保存先 ----
bag_topics = [
    # センサまわり
    '/sensors/R_force_N_raw',
    '/sensors/R_pos_mm',
    '/sensors/R_pres_kpa',
    '/targets/force_N',

    '/actuators/valve_voltage',
    #筋電を取るときは/ai1616llpe/voltageだけをのこしてほかは全部コメントアウトにすること！！
    '/ai1616llpe/voltage',
]

bag_base_dir = os.path.expanduser('~/koni_ws/src/py_force_controller/log')
os.makedirs(bag_base_dir, exist_ok=True)

stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
bag_out = os.path.join(bag_base_dir, f'force_bag_{stamp}')

bag_record = ExecuteProcess(
    cmd=['ros2', 'bag', 'record', '-o', bag_out, *bag_topics],
    output='screen'
)

def generate_launch_description():
    return LaunchDescription([

        # AI
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='AI_board',
            output='screen',
            parameters=[{'update_rate': 1000.0}],
        ),

#電圧→物理量変換 

        Node(
            package='py_signal_processing',
            executable='analog_voltage_interpreter',
            name='analog_voltage_interpreter',
            output='screen',
            parameters=[{
                'input_topic': '/ai1616llpe/voltage',
                'ai_channels': 16,
                # [L_pos, R_pos, L_pres, R_pres, L_load, R_load]
                'index_map': [1, 2, 5, 4, 7, 6],
                'pos_fullscale_v': 5.0, 'pos_fullscale_mm': 1000.0,
                'pres_vmin': 1.0, 'pres_vmax': 5.0,
                'pres_rmin_kpa': 0.0, 'pres_rmax_kpa': 1000.0,

                # 2025/09/30 時点の換算
                # 'load_v_zero_N': -0.075,
                'load_v_zero_N': 0.0,
                # 'load_gain_N_per_v': 116.6196216,
                #'load_gain_N_per_v': 184.1667,
                'load_gain_L_N_per_v': 199.07,  # ← 左
                'load_gain_R_N_per_v': 181.65, # ← 右 
                'publish_kg_topics': False,
            }],
        ),

        # 位置に応じた可変負荷: F_ref = ax + b 
        Node(
            package='py_force_controller',
            executable='variable_force_controller', 
            name='variable_force',
            output='screen',
            parameters=[{
                # ループとPI
                'rate_hz': 1000.0,
                'kp': 0.08,
                'ki': 0.12, #0.09
                'i_limit': 30.0,

                # 電圧レンジ
                'neutral_v': 5.0,
                'vmin': 0.1,
                'vmax': 9.9,

                'sensor_is_kgf': False,

                # 力LPF
                'force_lpf_hz': 10.0,

                'topic_force_L': '/sensors/L_force_N_raw',
                'topic_force_R': '/sensors/R_force_N_raw',
                'topic_out':     '/actuators/valve_voltage',

                # F = a*x + b の設定 
                # 50N→200Nなら、y = -0.89x + 328
                # 100N→300Nなら、y = -1.136x + 461.36
                # 50N→350Nなら。a = -2.278 b = 819.25
                'use_affine_target': True,
                'pos_topic': '/sensors/R_pos_mm',  # 位置mm（床基準で“糸が伸びるほど値↑）
                'pos_lpf_hz': 10.0,
                'a_N_per_mm': -0.89,
                'b_N': 328.0,  # 切片N,
                'target_min_N': 50.0, #50
                'target_max_N': 200.0, #400
            }],
        ),

        Node(
            package='control_box',
            executable='ao1608llpe_test',
            name='AO_board',
            output='screen',
            parameters=[{
                'update_rate': 1000.0,

                'subscribe_topic': '/actuators/valve_voltage',
            }],
       ),
        # レコード
  #bag_record,
    ])
