from launch import LaunchDescription
from launch_ros.actions import Node
import os, datetime
from launch.actions import ExecuteProcess

# 保存したいトピック）
bag_topics = [
    '/sensors/L_force_N_raw',
    '/sensors/R_force_N_raw',
    '/actuators/valve_voltage',
    '/debug/target_force_N',
    '/sensors/L_force_N_filtered',
    '/sensors/R_force_N_filtered',
]

#保存先
bag_base_dir = os.path.expanduser('~/koni_ws/src/py_force_controller/log')
os.makedirs(bag_base_dir, exist_ok=True)  

# ディレクトリ名
stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
bag_out = os.path.join(bag_base_dir, f'force_bag_{stamp}')

# ros2bag record 
bag_record = ExecuteProcess(
    cmd=['ros2', 'bag', 'record', '-o', bag_out, *bag_topics],
    output='screen'
)
def generate_launch_description():
    return LaunchDescription([
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
                #'load_v_zero_N': -0.075, 
                'load_v_zero_N': 0, 
                #'load_gain_N_per_v': 116.6196216, 
                'load_gain_N_per_v': 184.1667,
                'publish_kg_topics':False
            }],
        ),
        Node(
            package='py_force_controller',
            executable='single_loop_force_control',
            name='force_ctrl',
            output='screen',
            parameters=[{
                # 制御レート＆PI
                'rate_hz': 500.0,
                'kp': 0.04,#0.08
                'ki': 0.07, #0.04
                'i_limit': 10.0,

                # AO電圧レンジ・ニュートラル
                'neutral_v': 5.0,
                'vmin': 0.1,
                'vmax': 9.9,

                # 目標値
                'sensor_is_kgf': False,
                'target_value': 50.0,  # sensor_is_kgf=True ならkg、False ならN

                # 力のローパス
                'force_lpf_hz': 15.0,

                #トピック
                'topic_force_L': '/sensors/L_force_N_raw',
                'topic_force_R': '/sensors/R_force_N_raw',
                'topic_out':     '/actuators/valve_voltage',
                'topic_target_pub': '/debug/target_force_N',
                'topic_forceL_N_filt': '/sensors/L_force_N_filtered',
                'topic_forceR_N_filt': '/sensors/R_force_N_filtered',
            }],
        
        ),
            # AO
        Node(
            package='control_box',
            executable='ao1608llpe_test',
            name='AO_board',
            output='screen',
            parameters=[{'update_rate': 1000.0}],
        ),
        bag_record,
    ])
