#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

int fd = -1;
char buf[32];
unsigned short int analogData[16];

float update_rate;

int BoardOpen()
{
  std::cout << "AI Board Open" << "\n";
  fd = open("/dev/AI", O_RDWR);
  if(fd == -1)
  {
    std::cout << "fopen error: AI-1616L-LPE" << "\n";
    // RCLCPP_INFO(node->get_logger(), "fopen error: AI-1616L-LPE");
    return -1;
  }
  return 0;
}

int BoardUpdate()
{
  char* bufptr = buf;

  if(read(fd, buf, sizeof(buf)) == -1)
  {
    std::cout << "Update error: AI-1616L-LPE" << "\n";
    return -1;
  }

  bufptr = buf;

  for(int i = 0; i < 16; i++)
  {
    analogData[i] = *(unsigned short int*)bufptr;
    bufptr += sizeof(unsigned short int);	
  }
  return 0;
}

int BoardClose()
{
  std::cout << "AI Board Close" << "\n";
  close(fd);
  return 0;
}

class AIPublisher : public rclcpp::Node
{
public:
  AIPublisher()
  : Node("ai1616llpe"), count_(0)
  {
    this->declare_parameter<float>("update_rate", 1000.0); 
    this->get_parameter("update_rate", update_rate);

    sampling_time_micros = std::chrono::microseconds(lround(1000000.0/update_rate)) ;   // [ms]

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("ai1616llpe/voltage", 10);
    timer_ = this->create_wall_timer(
      sampling_time_micros, std::bind(&AIPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.clear();
    BoardUpdate();

    for(int i = 0; i < 16; i++)
    {
        double value = (analogData[i]-65535.0/2.0)/65535.0*20.0;
        msg.data.push_back(value);
    }

    publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  size_t count_;
  std::chrono::microseconds sampling_time_micros;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  BoardOpen();  

  rclcpp::spin(std::make_shared<AIPublisher>());

  BoardClose();

  rclcpp::shutdown();
  return 0;
}
