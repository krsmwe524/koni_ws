# cylinder_exp パッケージ

空気圧シリンダの位置制御実験用パッケージです。  
カスケード制御（外側：位置PID／内側：圧力PI）によって、シリンダを指定した正弦波軌道に追従させます。

---

## ノード構成

本パッケージ単体では動作せず、`control_box` および `py_signal_processing` パッケージと組み合わせて使用します。  
`cylinder_control.launch.py` を使うと以下の5ノードが一括起動されます。

```
[control_box]          ai1616llpe_test      : AIボード（アナログ入力）ドライバ
[control_box]          ao1608llpe_test      : AOボード（アナログ出力）ドライバ
[control_box]          cnt3204mtlpe_test    : カウンタボード（エンコーダ）ドライバ
[py_signal_processing] analog_voltage_interpreter_cyl : センサ値変換・フィルタ
[cylinder_exp]         pos_controller       : 位置制御ノード（本パッケージ）
```

---

## トピック一覧

### ハードウェアドライバ層

| トピック名 | 型 | 向き | 発行ノード | 内容 |
|---|---|---|---|---|
| `/ai1616llpe/voltage` | `Float32MultiArray` | publish | `ai1616llpe_test` | AIボード 16ch 生電圧 [V] |
| `/cnt3204mtlpe/count` | `Float32MultiArray` | publish | `cnt3204mtlpe_test` | エンコーダカウント値（4ch） |
| `/actuators/valve_voltage` | `Float32MultiArray` | subscribe | `ao1608llpe_test` | バルブ指令電圧 [V]（8ch、中立=5V） |

### センサ変換層（`analog_voltage_interpreter_cyl`）

| トピック名 | 型 | 向き | 内容 |
|---|---|---|---|
| `/sensors/cylinder_position` | `Float32` | publish | シリンダ位置（エンコーダ換算）[m]、LPF済み |
| `/sensors/head_pressure`     | `Float32` | publish | ヘッド側圧力 [kPa]、LPF済み |
| `/sensors/rod_pressure`      | `Float32` | publish | ロッド側圧力 [kPa]、LPF済み |
| `/sensors/loadcell_force`    | `Float32` | publish | ロードセル計測力 [N]、LPF済み（現在は制御未使用） |

### 制御層（`pos_controller`）

| トピック名 | 型 | 向き | 内容 |
|---|---|---|---|
| `/sensors/cylinder_position` | `Float32` | subscribe | シリンダ位置（センサ変換層から） |
| `/sensors/head_pressure`     | `Float32` | subscribe | ヘッド側圧力（センサ変換層から） |
| `/sensors/rod_pressure`      | `Float32` | subscribe | ロッド側圧力（センサ変換層から） |
| `/actuators/valve_voltage`   | `Float32MultiArray` | publish | バルブ指令電圧 [V]（8ch） |
| `/control/target_position`   | `Float32` | publish | 目標位置（ランプ軌道または正弦波）[m]（PlotJuggler確認用） |
| `/control/current_rel_position` | `Float32` | publish | 現在の相対位置 [m]（原点からの変位）（PlotJuggler確認用） |
| `/debug/cylinder_controller` | `Float32MultiArray` | publish | デバッグ情報（下記参照） |

#### `/debug/cylinder_controller` の配列インデックス

| インデックス | 内容 | 単位 |
|---|---|---|
| [0] | 目標ヘッド圧 | kPa |
| [1] | 現在ヘッド圧 | kPa |
| [2] | 目標ロッド圧 | kPa |
| [3] | 現在ロッド圧 | kPa |
| [4] | ヘッド側バルブ加算電圧 | V |
| [5] | ロッド側バルブ加算電圧 | V |
| [6] | 目標推力 | N |

---

## 制御構成

```
センサ入力
  /sensors/cylinder_position [m]
  /sensors/head_pressure     [kPa]
  /sensors/rod_pressure      [kPa]
         │
         ▼
  ┌──────────────────────────────────────┐
  │ 外側ループ（outer_rate_hz: 500Hz）    │
  │   PRESSURIZE: ランプ軌道追従          │
  │   RUNNING:    正弦波追従              │
  │   位置PID → 目標推力 [N]             │
  └──────────────────┬───────────────────┘
                     │ _target_force_N
                     ▼
  ┌──────────────────────────────────────┐
  │ 内側ループ（inner_rate_hz: 1000Hz）   │
  │   推力 → 目標圧力（片側加圧方式）     │
  │   圧力PI → バルブ加算電圧 [V]        │
  └──────────────────┬───────────────────┘
                     │
                     ▼
  /actuators/valve_voltage（中立5V ± u）
```

### 起動シーケンス

```
WAITING_SENSOR  センサ到着待ち（バルブ中立）
      ↓ センサ受信
HOMING          ロッド側全開（Head=0V, Rod=10V）で機械原点まで押し切る
                位置変化が homing_settle_threshold 以下で homing_settle_duration 秒継続 → 原点確定
      ↓ 原点確定
PRESSURIZE      ランプ軌道（ramp_rate_m_s）で center_position_m まで移動
                中心との誤差が pressurize_pos_threshold 以下で pressurize_settle_duration 秒継続 → 遷移
      ↓ 中心到達・安定
RUNNING         正弦波軌道（center ± amplitude × sin(2π × freq × t)）を追従
```

---

## 起動方法

```bash
ros2 launch cylinder_exp cylinder_control.launch.py
```

---

## パラメータ一覧

### ハードウェア

| パラメータ | デフォルト | 単位 | 説明 |
|---|---|---|---|
| `ch_head` | `0` | - | AOボード ヘッド側チャンネル番号 |
| `ch_rod`  | `1` | - | AOボード ロッド側チャンネル番号 |

### ループ周期

| パラメータ | デフォルト | 単位 | 説明 |
|---|---|---|---|
| `outer_rate_hz` | `500.0` | Hz | 外側（位置）ループ周期 |
| `inner_rate_hz` | `1000.0` | Hz | 内側（圧力）ループ周期 |

> Python + rclpy で安定して回せる周期は実機で実測して確認すること。

### 正弦波軌道

| パラメータ | デフォルト | 単位 | 説明 |
|---|---|---|---|
| `center_position_m`  | `0.005` | m | 正弦波の中心位置（原点からの相対） |
| `sine_amplitude_m`   | `0.004` | m | 正弦波の振幅 |
| `sine_freq_hz`       | `0.5`   | Hz | 正弦波の周波数 |

### PRESSURIZE ランプ軌道

| パラメータ | デフォルト | 単位 | 説明 |
|---|---|---|---|
| `ramp_rate_m_s` | `0.005` | m/s | 中心位置へ近づく速度。小さいほど振動しにくい。 |

### 圧力設定

| パラメータ | デフォルト | 単位 | 説明 |
|---|---|---|---|
| `base_pressure_kpa`   | `150.0` | kPa | 両室のベース圧力 |
| `supply_pressure_kpa` | `600.0` | kPa | 圧力指令の上限（供給圧以下に設定） |

### 位置ループ PID（外側）

| パラメータ | デフォルト | 単位 | 説明 |
|---|---|---|---|
| `pos_kp`           | `2000.0` | N/m     | 比例ゲイン |
| `pos_ki`           | `0.0`    | N/(m·s) | 積分ゲイン |
| `pos_kd`           | `0.0`    | N·s/m   | 微分ゲイン |
| `pos_td`           | `0.05`   | s       | 微分フィルタ時定数 |
| `pos_output_limit` | `1000.0` | N       | 推力指令の上限（アンチワインドアップ上限にも使用） |

### 圧力ループ PI（内側）

| パラメータ | デフォルト | 単位 | 説明 |
|---|---|---|---|
| `pres_kp`           | `0.01` | V/kPa     | 比例ゲイン |
| `pres_ki`           | `0.01` | V/(kPa·s) | 積分ゲイン |
| `pres_kd`           | `0.0`  | -         | 微分ゲイン（通常0） |
| `pres_td`           | `0.01` | s         | 微分フィルタ時定数 |
| `pres_output_limit` | `4.9`  | V         | バルブ加算電圧の上限。中立5V ± 4.9V → 0.1〜9.9V に収まる。 |

### ホーミング

| パラメータ | デフォルト | 単位 | 説明 |
|---|---|---|---|
| `homing_settle_threshold` | `0.0002` | m | 位置変化がこれ以下で停止とみなす |
| `homing_settle_duration`  | `1.0`    | s | 停止判定の継続時間 |
| `homing_startup_wait`     | `0.5`    | s | 起動直後の待機時間 |

### PRESSURIZE → RUNNING 遷移

| パラメータ | デフォルト | 単位 | 説明 |
|---|---|---|---|
| `pressurize_pos_threshold`   | `0.002` | m | 中心との誤差がこれ以下で収束とみなす |
| `pressurize_settle_duration` | `0.5`   | s | 収束判定の継続時間 |

---

## センサ変換パラメータ（`analog_voltage_interpreter_cyl`）

| パラメータ | デフォルト | 説明 |
|---|---|---|
| `head_pressure_index`   | `0`     | AIボードのヘッド圧チャンネル番号 |
| `rod_pressure_index`    | `1`     | AIボードのロッド圧チャンネル番号 |
| `encoder_index`         | `0`     | カウンタボードのエンコーダチャンネル番号 |
| `meters_per_count`      | `0.00004` | エンコーダ1カウントあたりの距離 [m] |
| `slope_kPa_per_V_head`  | `250.0` | ヘッド圧センサの感度 [kPa/V] |
| `slope_kPa_per_V_rod`   | `250.0` | ロッド圧センサの感度 [kPa/V] |
| `cutoff_hz_encoder`     | `20.0`  | 位置LPFカットオフ [Hz] |
| `cutoff_hz_pressure`    | `20.0`  | 圧力LPFカットオフ [Hz] |
| `cutoff_hz_loadcell`    | `5.0`   | ロードセルLPFカットオフ [Hz] |
| `kg_per_V_loadcell`     | `7.9186`| ロードセル感度 [kg/V] |
