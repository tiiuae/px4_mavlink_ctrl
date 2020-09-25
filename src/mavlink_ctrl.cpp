#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <common/common.hpp>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

class MavlinkCtrl : public rclcpp::Node
{
  public:
    MavlinkCtrl()
    : Node("MavlinkCtrl"),
      count_(0),
      buffer_(2041, 0),
      bytes_sent_(0)
    {
      this->declare_parameter<int>("udp_local_port", 1999);
      this->declare_parameter<int>("udp_remote_port", 2000);
      this->declare_parameter<std::string>("target_ip", "127.0.0.1");
      this->get_parameter("udp_local_port", udp_local_port_);
      this->get_parameter("udp_remote_port", udp_remote_port_);
      this->get_parameter("target_ip", target_ip_);

      RCLCPP_INFO(this->get_logger(), "udp_local_port  : %d", udp_local_port_);
      RCLCPP_INFO(this->get_logger(), "udp_remote_port_: %d", udp_remote_port_);
      RCLCPP_INFO(this->get_logger(), "target_ip       : '%s'", target_ip_.c_str());

      //buffer(BUFFER_LENGTH, 0);
      memset(&locAddr_, 0, sizeof(locAddr_));
      locAddr_.sin_family = AF_INET;
      locAddr_.sin_addr.s_addr = INADDR_ANY;
      locAddr_.sin_port = htons(udp_local_port_);

      /* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */
      if (-1 == bind(sock_,(struct sockaddr *)&locAddr_, sizeof(struct sockaddr)))
      {
        perror("error bind failed");
        close(sock_);
        exit(EXIT_FAILURE);
      }

      /* Attempt to make it non blocking */
    #if (defined __QNX__) | (defined __QNXNTO__)
    	if (fcntl(sock_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    #else
    	if (fcntl(sock_, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
    #endif
      {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sock_);
        exit(EXIT_FAILURE);
      }

      memset(&gcAddr_, 0, sizeof(gcAddr_));
      gcAddr_.sin_family = AF_INET;
      gcAddr_.sin_addr.s_addr = inet_addr(target_ip_.c_str());
      gcAddr_.sin_port = htons(udp_remote_port_);

      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "mavlinkcmd", 10, std::bind(&MavlinkCtrl::topic_callback, this, _1));

    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      /*Send NAV cmd */
      mavlink::common::msg::COMMAND_LONG cmd;
      mavlink::mavlink_message_t mavmsg;

      uint8_t *buf = (uint8_t*) buffer_.data();

      cmd.target_system = 1;
      cmd.target_component = 1;
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      if (strcmp(msg->data.c_str(), "takeoff") == 0)
      {
        RCLCPP_INFO(this->get_logger(), "takeoff cmd");
        cmd.command = 22;
        cmd.param1 = -1;
        cmd.param2 = 0;
        cmd.param3 = 0;
        cmd.param4 = std::nan("0");
        cmd.param5 = std::nan("0");
        cmd.param6 = 0;
        cmd.param7 = 500;

        mavlink::MsgMap map(mavmsg);
        cmd.serialize(map);
        mavlink_finalize_message(&mavmsg, 255, 0, 33, 33, 152);
        int len = mavlink_msg_to_send_buffer(buf, &mavmsg);
        bytes_sent_ = sendto(sock_, buf, len, 0, (struct sockaddr*)&gcAddr_, sizeof(struct sockaddr_in));
        RCLCPP_INFO(this->get_logger(), "bytes_sent: %d", bytes_sent_);

        cmd.command = 400;
        cmd.param1 = 1;
        cmd.param7 = 0;

        cmd.serialize(map);
        mavlink_finalize_message(&mavmsg, 255, 0, 33, 33, 152);
        len = mavlink_msg_to_send_buffer(buf, &mavmsg);
        bytes_sent_ = sendto(sock_, buf, len, 0, (struct sockaddr*)&gcAddr_, sizeof(struct sockaddr_in));
        RCLCPP_INFO(this->get_logger(), "bytes_sent: %d", bytes_sent_);

      }
      else if (strcmp(msg->data.c_str(), "land") == 0)
      {
        cmd.command = 21;
        cmd.param1 = 0;
        cmd.param2 = 0;
        cmd.param3 = 0;
        cmd.param4 = 0;
        cmd.param5 = 0;
        cmd.param6 = 0;
        cmd.param7 = 10;
        mavlink::MsgMap map(mavmsg);
        cmd.serialize(map);
        mavlink_finalize_message(&mavmsg, 255, 0, 33, 33, 152);
        int len = mavlink_msg_to_send_buffer(buf, &mavmsg);
        bytes_sent_ = sendto(sock_, buf, len, 0, (struct sockaddr*)&gcAddr_, sizeof(struct sockaddr_in));
        RCLCPP_INFO(this->get_logger(), "bytes_sent: %d", bytes_sent_);
      }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    const int BUFFER_LENGTH = 2041;
    size_t count_;
    uint16_t udp_local_port_;
    uint16_t udp_remote_port_;
    std::string target_ip_;

  	int sock_ = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  	struct sockaddr_in gcAddr_;
  	struct sockaddr_in locAddr_;
  	//struct sockaddr_in fromAddr;
  	std::vector<uint8_t> buffer_;
    uint32_t bytes_sent_;
    std::string udp_l_string_;
    std::string udp_r_string_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavlinkCtrl>());
  rclcpp::shutdown();
  return 0;
}
