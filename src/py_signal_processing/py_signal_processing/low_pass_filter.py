import math

class LowPassFilter:
    """
    リアルタイム処理用の1次ローパスフィルタ
    """
    def __init__(self, cutoff_hz):
        self.cutoff_hz = cutoff_hz
        self.prev_time = None
        self.prev_y = None

    def update(self, current_val, current_time_sec):
        # カットオフ周波数が0以下の場合はフィルタをかけずにそのまま返す（バイパス）
        if self.cutoff_hz <= 0.0:
            return current_val

        # 初回実行時は初期化だけしてそのままの値を返す
        if self.prev_time is None or self.prev_y is None:
            self.prev_time = current_time_sec
            self.prev_y = current_val
            return current_val

        # 前回からの経過時間(dt)を計算
        dt = current_time_sec - self.prev_time
        if dt <= 0.0:
            return self.prev_y

        # RCフィルタの離散化計算 (y[i] = y[i-1] + α * (x[i] - y[i-1]))
        rc = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        alpha = dt / (rc + dt)
        
        # フィルタリングされた新しい値
        y = self.prev_y + alpha * (current_val - self.prev_y)
        
        # 次回のために状態を保存
        self.prev_time = current_time_sec
        self.prev_y = y
        
        return y