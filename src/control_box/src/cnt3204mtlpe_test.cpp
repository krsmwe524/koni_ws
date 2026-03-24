#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <iostream>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <chrono>
#include <cmath>

using namespace std_msgs::msg;
using namespace std::chrono_literals;

char buf[128];
int fd = -1;
uint32_t counterData[4];

int flag = 0;
uint32_t initial[4];
int cnt_indexes;

int BoardOpen()
{
    std::cout << "CNT Board Open\n";
    fd = open("/dev/CNT", O_RDWR);
    if(fd == -1) {
        std::cout << "fopen error: CNT-3204MT-LPE\n";
        return -1;
    }
    return 0;
}

int BoardClose()
{
    std::cout << "CNT Board Close\n";
    if(fd != -1) close(fd);
    return 0;
}

int BoardUpdate()
{
    if(read(fd, buf, sizeof(buf)) == -1)
    {
        std::cout << "Update error: CNT-3204MT-LPE\n";
        return -1;
    }
    
    char* bufptr = buf;
    for(int i=0; i<4; i++) {
        counterData[i] = *(uint32_t*)bufptr;        
        bufptr += sizeof(uint32_t);
    }
    return 0;
}

// ROS 2 ノード
class CNTNode : public rclcpp::Node
{
public:
  CNTNode() : Node("cnt3204mtlpe")
  {

    this->declare_parameter<int>("cnt_indexes", 4);
    this->get_parameter("cnt_indexes", cnt_indexes);

    this->declare_parameter<float>("update_rate", 1000.0);
    float update_rate;
    this->get_parameter("update_rate", update_rate);

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/cnt3204mtlpe", 10);

    auto sampling_time_micros = std::chrono::microseconds(lround(1000000.0 / update_rate));
    timer_ = this->create_wall_timer(
      sampling_time_micros, std::bind(&CNTNode::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "CNT Node started. Publishing raw pulses to /cnt3204mtlpe at %.1f Hz", update_rate);
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.clear();

    BoardUpdate();

    for(int i = 0; i < cnt_indexes; i++)
    {
      uint32_t value = counterData[i];

      // 初期位置の記録
      if (flag == 0){
        initial[i] = value;
        if (i == cnt_indexes - 1){
          flag = 1;
        }
      }

      int32_t signed_diff = static_cast<int32_t>(value - initial[i]);
      double diff_count = static_cast<double>(signed_diff);
      
      msg.data.push_back(diff_count);
    }

    publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (BoardOpen() != 0) {
      rclcpp::shutdown();
      return -1;
  }

  rclcpp::spin(std::make_shared<CNTNode>());

  BoardClose();
  rclcpp::shutdown();
  return 0;
}