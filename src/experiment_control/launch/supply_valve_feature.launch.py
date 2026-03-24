from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # AO 出力ノード
    ao_node = Node(
        package='control_box',
        executable='ao1608llpe_test',
        name='AO_board',
        output='screen',
        parameters=[
            {'subscribe_topic': '/actuators/valve_voltage'},
            {'channel': 1},
            {'update_rate': 1000.0},]
    )

    # AI 入力ノード
    ai_node = Node(
        package='control_box',
        executable='ai1616llpe_test',
        name='AI_board',
        output='screen',
        # もし /ai1616llpe/voltage の QoS や何かパラメータがあればここに
        parameters=[{'update_rate': 1000.0}]
    )

    # バルブ給気側特性計測ノード
    feature_node = Node(
        package='experiment_control',
        executable='supply_valve_feature',
        name='supply_valve_feature',
        output='screen',
        parameters=[
            # 開口特性を取りたい電圧リスト
            {'u_list': [5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5]},
            # 中立電圧
            {'u_neutral': 5.0},
            # 1ステップあたりの待ち時間・計測時間 [s]
            {'settling_duration': 1.0},
            {'measure_duration': 1.0},
            # サンプリング周期1000 Hz
            {'control_dt': 0.001},
            # AI のチャンネル割り当て
            {'flow_ch': 12},
            {'psup_ch': 3},
            {'pdown_ch': 2},
            # ログの有無と保存先
            {'enable_logging': True},
            {'output_csv': 'valve_feature'},
            # トピック名
            {'valve_cmd_topic': '/actuators/valve_voltage'},
            {'ai_topic': '/ai1616llpe/voltage'},
        ]
    )

    return LaunchDescription([
        ao_node,
        ai_node,
        feature_node,
    ])