from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. AIボード ドライバ (センサ入力: 16ch電圧)
        # C++のノード: control_box / ai1616llpe
        Node(
            package='control_box',
            executable='ai1616llpe',
            name='ai_driver',
            output='screen'
        ),

        # 2. AOボード ドライバ (バルブ出力: 指令電圧受信)
        # C++のノード: control_box / ao1608llpe
        Node(
            package='control_box',
            executable='ao1608llpe',
            name='ao_driver',
            output='screen'
        ),

        # 3. 物理量変換ノード (電圧 -> mm, kPa, N)
        # Pythonのノード: py_signal_processing (または control_box)
        # ここでパラメータ 'index_map' を上書きできます！
        Node(
            package='control_box',  # 実際にあるパッケージ名を指定してください
            executable='analog_voltage_interpreter',
            name='interpreter',
            output='screen',
            parameters=[{
                # 以前の質問にあった「新しいセンサ(10番)」を含めた設定をここで記述できます
                # コードを書き換えずに、Launchだけで設定変更できるのがメリットです
                'index_map': [1, 2, 5, 4, 7, 6, 10],
                'pos_fullscale_mm': 1000.0,
                # 必要に応じて他のパラメータもここに書けます
            }]
        ),

        # 4. データロガー (sensingパッケージ)
        # すべてのデータをCSVに記録
        Node(
            package='sensing',
            executable='data_logger',
            name='data_logger',
            output='screen',
            parameters=[{
                'log_dir': './log_experiment' # ログの保存先を指定
            }]
        ),
    ])