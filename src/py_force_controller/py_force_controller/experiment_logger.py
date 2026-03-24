                        import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import csv
import os
from datetime import datetime

class ExperimentLogger(Node):
    def __init__(self):
        super().__init__('experiment_logger')
        log_dir = "/home/kklab/koni_ws/src/py_force_controller/prbs_log"
        os.makedirs(log_dir, exist_ok=True)

        self.filepath = os.path.join(log_dir, datetime.now().strftime("pam_prc_%Y%m%d_%H%M%S.csv"))
        # buffering=1 (行単位で即時保存)
        self.csv_file = open(self.filepath, 'w', newline='', buffering=1)
        self.writer = csv.writer(self.csv_file)
        
        # ヘッダー 
        self.writer.writerow(['t_ros', 'u_cmd_L', 'u_cmd_R', 'p_ch_L', 'p_ch_R', 'p_sup', 'L_pos_mm', 'R_pos_mm', 'F_L', 'F_R'])
        self.data = {k: 0.0 for k in ['uL', 'uR', 'pL', 'pR', 'ps', 'posL', 'posR', 'fL', 'fR']}

        self.create_subscription(Float32MultiArray, '/actuators/valve_voltage', self.cb_u, 10)
        self.create_subscription(Float32, '/sensors/L_pres_kpa', lambda m: self.set_val('pL', m), 10)
        self.create_subscription(Float32, '/sensors/R_pres_kpa', lambda m: self.set_val('pR', m), 10)
        self.create_subscription(Float32, '/sensors/sup_press', lambda m: self.set_val('ps', m), 10)
        self.create_subscription(Float32, '/sensors/L_pos_mm', lambda m: self.set_val('posL', m), 10)
        self.create_subscription(Float32, '/sensors/R_pos_mm', lambda m: self.set_val('posR', m), 10)
        self.create_subscription(Float32, '/sensors/L_force_N_raw', lambda m: self.set_val('fL', m), 10)
        self.create_subscription(Float32, '/sensors/R_force_N_raw', lambda m: self.set_val('fR', m), 10)

        self.create_timer(0.001, self.log_timer_callback)
        self.flush_count = 0
        self.get_logger().info(f'Logging to: {self.filepath}')

    def set_val(self, key, msg):
        self.data[key] = msg.data

    def cb_u(self, msg):
        if len(msg.data) >= 2:
            self.data['uL'], self.data['uR'] = msg.data[0], msg.data[1]

    def log_timer_callback(self):
        t_now = self.get_clock().now().nanoseconds / 1e9
        row = [
        t_now, self.data['uL'], self.data['uR'],
        self.data['pL'], self.data['pR'], self.data['ps'],
        self.data['posL'], self.data['posR'], self.data['fL'], self.data['fR']
    ]
        self.writer.writerow(row)
        self.flush_count += 1
        if self.flush_count >= 100:
            self.csv_file.flush()
            self.flush_count = 0

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()