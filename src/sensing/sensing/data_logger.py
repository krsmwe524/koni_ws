import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv
import time
import os
from datetime import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # パラメータ: 保存ファイル名など
        self.declare_parameter('log_dir', './log')
        self.log_dir = self.get_parameter('log_dir').value
        
        # 保存用変数の初期化
        self.current_sensor_voltage = [] # AIボードからの値(16ch)
        self.current_valve_cmd = []      # バルブへの指令値(8ch)
        
        # 1. センサ値の購読 (AIボードから)
        self.sub_sensors = self.create_subscription(
            Float32MultiArray,
            '/ai1616llpe/voltage',
            self._cb_sensors,
            10
        )

        # 2. 指令電圧の購読 (ExperimentノードからAOボードへ送られるデータ)
        # ※ ここで「AOボードに送られる値」と同じものをこのノードも受け取ります
        self.sub_cmds = self.create_subscription(
            Float32MultiArray,
            '/actuators/valve_voltage',
            self._cb_cmds,
            10
        )

        # CSVファイルの準備
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        filename = datetime.now().strftime('experiment_%Y%m%d_%H%M%S.csv')
        self.filepath = os.path.join(self.log_dir, filename)
        
        self.f = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.f)
        
        # ヘッダー作成 (Time, Cmd_0..7, Sensor_0..15)
        header = ['timestamp'] + [f'cmd_ch{i}' for i in range(8)] + [f'sensor_ch{i}' for i in range(16)]
        self.writer.writerow(header)

        # 記録用タイマー (例: 100Hz = 0.01s)
        # 受信タイミングがバラバラなので、一定周期で「その瞬間の最新値」を書き込む方式が安全です
        self.create_timer(0.01, self._timer_callback)

        self.get_logger().info(f"Logging started: {self.filepath}")

    def _cb_sensors(self, msg):
        self.current_sensor_voltage = msg.data

    def _cb_cmds(self, msg):
        self.current_valve_cmd = msg.data

    def _timer_callback(self):
        # 両方のデータが一度でも来ているか確認
        if not self.current_sensor_voltage or not self.current_valve_cmd:
            return

        # タイムスタンプ
        now = self.get_clock().now().nanoseconds / 1e9
        
        # データの整形 (足りない場合は0埋めなどの処理を入れるとなお良し)
        # cmdは可変長の可能性があるため、8個に固定する処理
        cmd_data = list(self.current_valve_cmd)
        if len(cmd_data) < 8:
            cmd_data += [0.0] * (8 - len(cmd_data))
        else:
            cmd_data = cmd_data[:8]

        row = [now] + cmd_data + list(self.current_sensor_voltage)
        
        self.writer.writerow(row)

    def destroy_node(self):
        self.f.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()