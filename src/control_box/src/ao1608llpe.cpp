#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <memory.h>
#include <math.h>

int fd = -1;
char buf[32];
unsigned short int analogData[16];
bool digitalData[4];
std_msgs::msg::Float32MultiArray msg_received;
rclcpp::Node::SharedPtr node = nullptr;

int BoardOpen()
{
    RCLCPP_INFO(node->get_logger(), "Board Open");
    fd = open("/dev/AO", O_RDWR);
    if(fd == -1)
    {
        return -1;
    }
    for(int i = 0; i < 4; i++)
        digitalData[i] = 1;

    return 0;
}

int BoardClose()
{
    RCLCPP_INFO(node->get_logger(), "Board Close");
    close(fd);
    return 0;
}

int SetAnalogOut(int channel, double v_out)
{
    int data;
    double voltbuf;
    v_out = fabs(v_out);
    voltbuf = fabs(v_out);
    if(voltbuf > 10.0)
        v_out = 10.0 * ((v_out >= 0.0) - (v_out < 0.0));
    data = (int)((v_out + 10.0) * 65535.0 / 20.0);
    if(channel < 0) return -1;
    if(channel >= 8) return -1;
    analogData[channel] = data;
    return 0;
}

int Update()
{
    char* bufptr = buf;
    unsigned short int dadata[9];
    unsigned char k = (digitalData[3]<<3) | (digitalData[2]<<2) | (digitalData[1]<<1) | (digitalData[0]);

    bufptr = buf;
    for(int i = 0; i < 8; i++) dadata[i] = (unsigned short int)analogData[i];
    dadata[8] = 0x0000000F & k;
    for(int i = 0; i < 8; i++)
    {
        memcpy(bufptr, &dadata[i], sizeof(unsigned short int));
        bufptr += sizeof(unsigned short int);
    }
    if(write(fd, buf, sizeof(buf)) == -1)
    {
        RCLCPP_INFO(node->get_logger(), "UpdateOut error: AO1608LLPE");
        return -1;
    }
    return 0;
}

void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) // const
{

    for(long unsigned int i = 0; i < msg->data.size(); i++)
    {
        SetAnalogOut(i, msg->data[i]);
    }
 
    Update();
}

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("ao1608llpe");

    std::string subscribe_topic;
    node->declare_parameter<std::string>("subscribe_topic", "/pressure_out_saturated");
    node->get_parameter("subscribe_topic", subscribe_topic);

    auto subscription = node->create_subscription<std_msgs::msg::Float32MultiArray>(
        subscribe_topic, 10, callback);

    BoardOpen();
    rclcpp::spin(node);

    node = nullptr;

    BoardClose();

    rclcpp::shutdown();

    return 0;
}
