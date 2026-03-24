#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <algorithm> 
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <cmath>
#include <iostream>

using std::placeholders::_1;

namespace
{
    // 出力範囲（バルブ仕様 0–10V、5V中立）
    constexpr double V_NEUTRAL = 5.0;
    constexpr double V_MIN = 0.0;
    constexpr double V_MAX = 10.0;

    // DACは±10Vレンジ(= -10..+10V を 0..65535 にマップ)。
    inline uint16_t volt_to_code_pm10(double v)
    {
        // vは実電圧（-10..+10の想定）。ここでは事前に 0..10V に丸める運用。
        double vv = std::clamp(v, -10.0, 10.0);
        // (v + 10) / 20 * 65535
        double code = (vv + 10.0) * (65535.0 / 20.0);
        long c = lrint(code);
        if (c < 0)
            c = 0;
        if (c > 65535)
            c = 65535;
        return static_cast<uint16_t>(c);
    }

} // namespace

// ===== 低レベル I/O =====
static int fd = -1;
static uint16_t analogData[16] = {0}; // 物理DACに送るコード値（0..65535）

int BoardOpen()
{
    std::cout << "AO Board Open\n";
    fd = open("/dev/AO", O_RDWR);
    if (fd == -1)
    {
        std::cerr << "open error: AO-1608L-LPE (/dev/AO)\n";
        return -1;
    }
    // 安全のため、全chを“中立5V”に初期化
    const uint16_t neutral_code = volt_to_code_pm10(V_NEUTRAL);
    for (int i = 0; i < 8; ++i)
        analogData[i] = neutral_code;

    ssize_t need = 8 * sizeof(uint16_t);
    if (write(fd, reinterpret_cast<char *>(analogData), need) == -1)
    {
        std::cerr << "initial UpdateOut error\n";
        return -1;
    }
    return 0;
}

int BoardClose()
{
    std::cout << "AO Board Close\n";
    if (fd != -1)
        close(fd);
    fd = -1;
    return 0;
}

// 0..10V 運用用に安全クランプしてから、±10V方式でDACコード化
void SetAnalogOutVector(const std::vector<float> &v_outs)
{
    const size_t n = std::min<size_t>(v_outs.size(), 8);
    for (size_t ch = 0; ch < n; ++ch)
    {
        double v_cmd = std::clamp<double>(v_outs[ch], V_MIN, V_MAX); // 0..10V
        analogData[ch] = volt_to_code_pm10(v_cmd);
    }
}

// 指定chだけ更新（メッセージが1要素の時などに使用）
void SetAnalogOutSingle(int channel, float v_cmd_in)
{
    if (channel < 0 || channel >= 8)
        return;
    double v_cmd = std::clamp<double>(v_cmd_in, V_MIN, V_MAX); // 0..10V
    analogData[channel] = volt_to_code_pm10(v_cmd);
}

// アナログ8ch分だけを正しいバイト数で送る
int Update()
{
    if (fd == -1)
        return -1;
    ssize_t need = 8 * sizeof(uint16_t); // 16B
    if (write(fd, reinterpret_cast<char *>(analogData), need) == -1)
    {
        std::cerr << "UpdateOut error: AO1608LLPE\n";
        return -1;
    }
    return 0;
}

class AONode : public rclcpp::Node
{
public:
    AONode() : Node("ao1608llpe")
    {
        // topic:/actuators/valve_voltage
        // channel: 単一要素を受けた場合の出力先ch
        this->declare_parameter<std::string>("subscribe_topic", "/actuators/valve_voltage");
        this->declare_parameter<int>("channel", 0);
        this->get_parameter("subscribe_topic", subscribe_topic_);
        this->get_parameter("channel", channel_);

        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            subscribe_topic_, rclcpp::QoS(10),
            std::bind(&AONode::topic_callback, this, _1));

        RCLCPP_INFO(this->get_logger(),
                    "AO node started. topic='%s', channel=%d (single-value mode). "
                    "Outputs clamped to [%.1f, %.1f] V, neutral=%.1f V.",
                    subscribe_topic_.c_str(), channel_, V_MIN, V_MAX, V_NEUTRAL);
    }

private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (!msg)
            return;
        const auto &v = msg->data;

        if (v.empty())
            return;

        // 受信サイズに応じてモードを自動判定:
        //  - 1要素: param 'channel' のみ更新
        //  - 2～8要素: 先頭から順にch0..に書き込み
        //  - 9要素以上: 先頭8要素を使用
        if (v.size() == 1)
        {
            SetAnalogOutSingle(channel_, v[0]);
        }
        else
        {
            std::vector<float> vv(v.begin(), v.end());
            SetAnalogOutVector(vv);
        }

        if (Update() != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "AO Update() failed");
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    int channel_{0};
    std::string subscribe_topic_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (BoardOpen() != 0)
    {
        // ボードが開けなければROSを回さず終了
        rclcpp::shutdown();
        return -1;
    }

    auto ao_node = std::make_shared<AONode>();
    rclcpp::spin(ao_node);

    BoardClose();
    rclcpp::shutdown();
    return 0;
}