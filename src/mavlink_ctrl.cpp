#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include <vector>
#include <common/common.hpp>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace mavlink::common;

class MavlinkCtrl : public rclcpp::Node
{
  public:
    MavlinkCtrl()
    : Node("MavlinkCtrl"),
      count_(0),
      buffer_(2041, 0),
      bytes_sent_(0)
    {
      this->declare_parameter<int>("udp_local_port", 14590);
      this->declare_parameter<int>("udp_remote_port", 14591);
      this->declare_parameter<std::string>("target_ip", "127.0.0.1");
      this->get_parameter("udp_local_port", udp_local_port_);
      this->get_parameter("udp_remote_port", udp_remote_port_);
      this->get_parameter("target_ip", target_ip_);

      RCLCPP_INFO(this->get_logger(), "udp_local_port  : %d", udp_local_port_);
      RCLCPP_INFO(this->get_logger(), "udp_remote_port_: %d", udp_remote_port_);
      RCLCPP_INFO(this->get_logger(), "target_ip       : '%s'", target_ip_.c_str());

      memset(&locAddr_, 0, sizeof(locAddr_));
      locAddr_.sin_family = AF_INET;
      locAddr_.sin_addr.s_addr = htonl(INADDR_ANY);
      locAddr_.sin_port = htons(udp_local_port_);

      /* Bind a socket to local address */
      if (bind(sock_,(struct sockaddr *)&locAddr_, sizeof(struct sockaddr)) < 0)
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

      wp_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "path", 10, std::bind(&MavlinkCtrl::waypoints_callback, this, _1));

    }

  private:

    enum PX4_CUSTOM_MAIN_MODE {
      PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
      PX4_CUSTOM_MAIN_MODE_ALTCTL,
      PX4_CUSTOM_MAIN_MODE_POSCTL,
      PX4_CUSTOM_MAIN_MODE_AUTO,
      PX4_CUSTOM_MAIN_MODE_ACRO,
      PX4_CUSTOM_MAIN_MODE_OFFBOARD,
      PX4_CUSTOM_MAIN_MODE_STABILIZED,
      PX4_CUSTOM_MAIN_MODE_RATTITUDE,
      PX4_CUSTOM_MAIN_MODE_SIMPLE /* unused, but reserved for future use */
    };

    enum PX4_CUSTOM_SUB_MODE_AUTO {
      PX4_CUSTOM_SUB_MODE_AUTO_READY = 1,
      PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF,
      PX4_CUSTOM_SUB_MODE_AUTO_LOITER,
      PX4_CUSTOM_SUB_MODE_AUTO_MISSION,
      PX4_CUSTOM_SUB_MODE_AUTO_RTL,
      PX4_CUSTOM_SUB_MODE_AUTO_LAND,
      PX4_CUSTOM_SUB_MODE_AUTO_RESERVED_DO_NOT_USE, // was PX4_CUSTOM_SUB_MODE_AUTO_RTGS, deleted 2020-03-05
      PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET,
      PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND
    };

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      /*Send NAV cmd */
      std::map<std::string, std::function<void()>>::const_iterator it;

      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

      if (strcmp(msg->data.c_str(), "takeoff") == 0)
      {
        this->do_takeoff();
      }
      else if (strcmp(msg->data.c_str(), "land") == 0)
      {
        this->do_land();
      }
      else if (strcmp(msg->data.c_str(), "start_mission") == 0)
      {
        this->do_start_mission();
      }
      else if (strcmp(msg->data.c_str(), "return_home") == 0)
      {
        this->do_return_to_launch();
      }
      else if (strcmp(msg->data.c_str(), "pause_mission") == 0)
      {
        this->do_hold();
      }
      else if (strcmp(msg->data.c_str(), "resume_mission") == 0)
      {
        this->do_continue_mission();
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Unknown command!!");
      }

    }

    void waypoints_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
		RCLCPP_INFO(this->get_logger(), "Got %d waypoints %s", msg->poses.size());
	}

    void send_msg(mavlink::Message &msg, std::string msg_name)
    {
      mavlink::mavlink_message_t mavmsg;
      uint8_t *buf = (uint8_t*) buffer_.data();
      mavlink::Message::Info info = msg.get_message_info();

      RCLCPP_INFO(this->get_logger(), "MAVLINK SEND: MSG %s", msg_name.c_str());

      mavlink::MsgMap map(mavmsg);
      msg.serialize(map);

      mavlink_finalize_message(&mavmsg, 254, 0, info.min_length, info.length, info.crc_extra);
      int len = mavlink_msg_to_send_buffer(buf, &mavmsg);
      bytes_sent_ = sendto(sock_, buf, len, 0, (struct sockaddr*)&gcAddr_, sizeof(struct sockaddr_in));
      RCLCPP_INFO(this->get_logger(), "bytes_sent: %d", bytes_sent_);
    }

    void send_cmd(mavlink::common::msg::COMMAND_LONG &cmd, std::string cmd_name)
    {
      RCLCPP_INFO(this->get_logger(), "MAVLINK SEND: CMD %s", cmd_name.c_str());
      cmd.target_system = 1;
      cmd.target_component = 1;
      RCLCPP_INFO(this->get_logger(), "CMD:%d tid:%d, tcom:%d, p1:%f p2:%f p3:%f p4:%f p5:%f p6:%f p7:%f",
      cmd.command, cmd.target_system, cmd.target_component, cmd.param1, cmd.param2, cmd.param3, cmd.param4, cmd.param5, cmd.param6, cmd.param7);

      this->send_msg(cmd, "CMD-" + cmd_name);
    }

    void do_takeoff()
    {
      mavlink::common::msg::COMMAND_LONG cmd = {};

      cmd.command = static_cast<uint16_t>( MAV_CMD::NAV_TAKEOFF );
      cmd.param1 = -1;
      cmd.param4 = std::nan("0");
      cmd.param5 = std::nan("0");
      cmd.param7 = 500;
      this->send_cmd(cmd, "takeoff");

      cmd.command = 400;
      cmd.param1 = 1;
      cmd.param7 = 0;
      this->send_cmd(cmd, "arm");
    }

    void do_land()
    {
      mavlink::common::msg::COMMAND_LONG cmd = {};
      cmd.command = static_cast<uint16_t>( MAV_CMD::NAV_LAND );
      cmd.param7 = 10;
      this->send_cmd(cmd, "land");
    }

    void do_start_mission()
    {
      mavlink::common::msg::COMMAND_LONG cmd = {};
      cmd.command = static_cast<uint16_t>( MAV_CMD::MISSION_START );
      cmd.param1 = 0;
      cmd.param2 = 0;
      this->send_cmd(cmd, "start mission");
    }

    void do_return_to_launch()
    {
      mavlink::common::msg::COMMAND_LONG cmd = {};
      cmd.command = static_cast<uint16_t>( MAV_CMD::NAV_RETURN_TO_LAUNCH );
      this->send_cmd(cmd, "return to launch");
    }

    void do_hold()
    {
      mavlink::common::msg::SET_MODE msg = {};
      msg.custom_mode = static_cast<uint8_t>(PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_POSCTL) << 16; // 0x30000
      msg.target_system = 1;
      msg.base_mode = 209.0;
      RCLCPP_INFO(this->get_logger(), "SET_MODE custom_mode: %f", (double)msg.custom_mode);
      this->send_msg(msg, "SET_MODE:hold");
    }

    void do_continue_mission()
    {
      mavlink::common::msg::SET_MODE msg = {};
      msg.custom_mode = (static_cast<uint8_t>(PX4_CUSTOM_MAIN_MODE::PX4_CUSTOM_MAIN_MODE_AUTO) << 16) |
                        (static_cast<uint8_t>(PX4_CUSTOM_SUB_MODE_AUTO::PX4_CUSTOM_SUB_MODE_AUTO_MISSION) << 24); // 0x4040000
      msg.target_system = 1;
      msg.base_mode = 217.0;
      RCLCPP_INFO(this->get_logger(), "SET_MODE custom_mode: %f", (double)msg.custom_mode);
      this->send_msg(msg, "SET_MODE:continue mission");
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr wp_subscription_;
	
    const int BUFFER_LENGTH = 2041;
    size_t count_;
    uint16_t udp_local_port_;
    uint16_t udp_remote_port_;
    std::string target_ip_;

  	int sock_ = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  	struct sockaddr_in gcAddr_;
  	struct sockaddr_in locAddr_;
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
