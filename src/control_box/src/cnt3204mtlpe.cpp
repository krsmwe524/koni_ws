#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <iostream>
// #include <conio. h>

// #include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <memory.h>
#include <stdint.h>

#include <chrono>

char buf[128];
char obuf[1];
int fd = -1;
uint32_t counterData[4];

rclcpp::Node::SharedPtr node = nullptr;


int BoardOpen()    // CounterInput()
{
    fd = open("/dev/CNT", O_RDWR);
    if(fd == -1) {
        // RCLCPP_INFO(node->get_logger(), "fopen error: CNT-3204MT-LPE");
        return -1;
    }
    return 0;
}

int BoardClose()    // ~CounterInput()
{
    close(fd);
    return 0;
}

char* BoardUpdate()    // UpdateIn()   カウンタ値を更新する。（制御ループで毎回呼び出す。）
{
    char* bufptr = buf;

    char* testptr;
    
    if(read(fd, buf, sizeof(buf)) == -1)
    {
        // RCLCPP_INFO(this->get_logger(), "Update error: CNT-3204MT-LPE");
        // return -1;
    }
    
    bufptr = buf;

    for(int i=0; i<4; i++) {
        counterData[i] = *(uint32_t*)bufptr;        

        if(i == 0) testptr = bufptr;

        bufptr += sizeof(uint32_t);
    }

    return testptr;
    // return 0;
}

int GetCounterValue(int Channel)    // カウンタ値を読む
{
    if(Channel<0) return -1;
    if(Channel>=4) return -1;
    return(counterData[Channel]);
}

void CounterReset(int Channel)    // カウンタを0にリセットする。
{
    if(Channel<0) return;
    if(Channel>=4) return;
    obuf[0] = 0x01 << Channel;
    write(fd, obuf, sizeof(obuf));
}

void CounterResetAll()    // 全チャンネルのカウンタを0にリセットする。
{
    obuf[0] = 0x0F;
    write(fd, obuf, sizeof(obuf));
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cnt3204mtlpe");

  auto cnt3204mtlpe_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("cnt3204mtlpe/count", 10);
  rclcpp::Rate loop_rate(10);   // Hz

  BoardOpen();

//   CounterResetAll();

  std_msgs::msg::Float32MultiArray msg;
  
  uint32_t value;

  while (rclcpp::ok())
  {
    msg.data.clear();
    // char* tes = BoardUpdate();

    // // address check
    // RCLCPP_INFO(node->get_logger(), "bufptr : '%x'", tes);

    // RCLCPP_INFO(node->get_logger(), "&bufptr : '%f'", *(uint32_t*)tes);

    // int stsdata = _inpd(tes + 0x0c);   // ステータス呼び出し
//    char* stptr = tes + 0x0c;   // ステータス呼び出し

    // RCLCPP_INFO(node->get_logger(), "status data : '%02x'", &stptr);

    for(int i = 0; i < 4; i++)
		{
			value = counterData[i];
			// double value = (analogData[i]-65535.0/2.0)/65535.0*20.0;
			msg.data.push_back(value);
		}

    // RCLCPP_INFO(node->get_logger(), "Publishing: '%f'", msg.data[0]);

    // RCLCPP_INFO(node->get_logger(), "obuf : '%x'", fd);

    // stsdata = inpd(address + 0x0c);   // ステータス読み出し

    cnt3204mtlpe_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  BoardClose();

  return 0;
}

