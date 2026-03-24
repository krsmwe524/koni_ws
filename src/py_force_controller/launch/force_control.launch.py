from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from datetime import datetime
def generate_launch_description():
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    base_dir = os.path.expanduser('~/koni_ws/src/py_force_controller/log')
    os.makedirs(base_dir, exist_ok=True)
    bag_dir = os.path.join(base_dir, f'force_{ts}')

    return LaunchDescription([
        # AI
        Node(
            package='control_box',
            executable='ai1616llpe_test',
            name='AI_board',
            output='screen',
            parameters=[{'update_rate': 1000.0}],
        ),

        # analog_voltage_interpreter
        Node(
            package='py_signal_processing',
            executable='analog_voltage_interpreter',
            name='analog_voltage_interpreter',
            output='screen',
            parameters=[{
                'input_topic': '/ai1616llpe/voltage',
                'ai_channels': 8,
                # [L_pos, R_pos, L_pres, R_pres, L_load, R_load]
                'index_map': [1, 2, 5, 4, 7, 6],
                'pos_fullscale_v': 5.0, 'pos_fullscale_mm': 1000.0,
                'pres_vmin': 1.0, 'pres_vmax': 5.0,
                'pres_rmin_kpa': 0.0, 'pres_rmax_kpa': 1000.0,

                #2025/09/30時点
                'load_v_zero_N': 0.0, 
                #'load_gain_N_per_v': 116.6196216, #これはもうだめ
                #'load_gain_N_per_v': 181.8181, #2025/10/14 1点校正
                'load_gain_N_per_v': 184.1667, #2.2kgのおもりと、4.0kgのおもりで2点補正

                'publish_kg_topics':False
            }],
        ),

#負荷設定ノード
Node(
  package='py_force_controller', 
  executable='force_controller',
  name='force_target_generator',
  parameters=[{
    'mode': 'constant',
    'constant_N': 50.0, #一定の負荷
    'rate_hz': 200.0,
  }],
  arguments=['--node-kind','target'],
),

#外側
Node(
  package='py_force_controller',
  executable='force_controller',
  name='force_outer',
  output='screen',
  parameters=[{

    'rate_hz': 500.0,
    # p_ffの定数ズレ補正
    'ff_alpha': 1.0, # 1.0
    #'ff_beta': 80.0, #kPa
    'ff_b0': 0.0,#デフォは20.3
    'ff_b1':0.0, #デフォは462.6
    'kp_F': 0.20, #kpa/N　0.40もうちょい挙げれそう
    'ki_F': 0.50, #0.40
    'i_limit_F': 1000.0, # kPa

    'F_lpf_hz': 30.0,
    'pos_lpf_hz': 10.0, #ワイヤセンサのフィルタ。強め

    # 参照圧のレンジ/圧力センサ 0–1.0MPa なので最大1000。
    'pmin_kpa': 0.0,
    'pmax_kpa': 750.0,

    #フィードフォワード項のパラメータ
    'D0_mm':20.0,#期直径
    'theta0_deg': 25.0,   #編み角
    'init_length_mm': 1500.0, # 収縮部の長さ
    'top_L_mm': 1520.0, #左のPAM上端からポテンショメータの下のところまで
    'top_R_mm': 1560.0,
    'pos_topic_L_mm': '/sensors/L_pos_mm',
    'pos_topic_R_mm': '/sensors/R_pos_mm',

    # 力の入力（N）。
    'use_filtered_force': False,
    'force_topic_L_raw':  '/sensors/L_force_N_raw',
    'force_topic_R_raw':  '/sensors/R_force_N_raw',
    'force_topic_L_filt': '/sensors/L_force_N_filtered',
    'force_topic_R_filt': '/sensors/R_force_N_filtered',

  }],
  arguments=['--node-kind','outer'],   # 同一実行ファイルから役割を切る場合
),

#内側ループ
Node(
  package='py_force_controller', executable='force_controller', name='pressure_inner',
  parameters=[{
    'rate_hz': 900.0, 'P_lpf_hz': 2.0,
    'kp_P': 0.14, 'ki_P': 0.08, 'i_limit_P': 15.0,
    'neutral_v': 5.0, 
    'vmin': 0.1, 'vmax': 9.9,
    'valve_cmd_topic': '/actuators/valve_voltage',
    'pres_topic_L': '/sensors/L_pres_kpa',
    'pres_topic_R': '/sensors/R_pres_kpa',
    'pref_topic':   '/targets/pressure_kpa',
  }],
  arguments=['--node-kind','inner'],
),

    # AO
        Node(
            package='control_box',
            executable='ao1608llpe_test',
            name='AO_board',
            output='screen',
            parameters=[{'update_rate': 1000.0}],
        ),

        # bag, #←記録しないときはコメントアウト！


    ])
