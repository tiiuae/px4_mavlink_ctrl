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
#include <cmath>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/info/info.h>

using namespace mavsdk;
using namespace std::placeholders; // for `_1`
using namespace std::chrono; // for seconds(), milliseconds()
using namespace std::this_thread; // for sleep_for()

class MavlinkCtrl : public rclcpp::Node
{
  public:
    MavlinkCtrl()
    : Node("MavlinkCtrl"),
      count_(0),
      buffer_(2041, 0),
      bytes_sent_(0)
    {
      this->declare_parameter<int>("udp_remote_port", 14540);
      this->declare_parameter<std::string>("target_ip", "127.0.0.1");
      this->get_parameter("udp_remote_port", udp_remote_port_);
      this->get_parameter("target_ip", target_ip_);

      RCLCPP_INFO(this->get_logger(), "udp_remote_port_: %d", udp_remote_port_);
      RCLCPP_INFO(this->get_logger(), "target_ip       : '%s'", target_ip_.c_str());

      ConnectionResult connection_result = mavsdk.add_udp_connection(target_ip_.c_str(),udp_remote_port_);
      if (connection_result != ConnectionResult::Success) {
          RCLCPP_ERROR(this->get_logger(), "Connection failed: %s", connection_result);
          exit(EXIT_FAILURE);
      }

      // Wait for connection to mav id 1
      bool connected = false;
      unsigned i;
      while(!connected) {
          for (i=0; i < mavsdk.systems().size(); i++) {
              if  (mavsdk.systems().at(i)->get_system_id() == 1) {
                  connected=true;
                  break;
              }
          }
          std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      RCLCPP_INFO(this->get_logger(), "Target connected");

      system = mavsdk.systems().at(i);
      action = std::make_shared<Action>(system);
      mission = std::make_shared<Mission>(system);
      telemetry = std::make_shared<Telemetry>(system);

#if 0 // TODO asynch notifications along these lines...
      auto new_system_promise = std::promise<std::shared_ptr<System>>{};
      auto new_system_future = new_system_promise.get_future();
      mavsdk.subscribe_on_new_system([&mavsdk, &new_system_promise]() {
                                         new_system_promise.set_value(mavsdk.systems().at(0));
                                         mavsdk.subscribe_on_new_system(nullptr);
                                     });
      
      system = new_system_future.get();
      
      system->subscribe_is_connected([](bool is_connected) {
                                         if (is_connected) {
                                             std::cout << "System has been discovered" << std::endl;
                                         } else {
                                             std::cout << "System has timed out" << std::endl;
                                         }
                                     });
#endif
	
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "mavlinkcmd", 10, std::bind(&MavlinkCtrl::topic_callback, this, _1));

      wp_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "path", 10, std::bind(&MavlinkCtrl::waypoints_callback, this, _1));

    }

  private:

    Mavsdk mavsdk;
    std::shared_ptr<System> system;
    std::shared_ptr<mavsdk::Action> action;
    std::shared_ptr<mavsdk::Mission> mission;
    std::shared_ptr<mavsdk::Telemetry> telemetry;

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

    void do_takeoff()
    {
        // Arm vehicle
        const Action::Result arm_result = action->arm();        
        if (arm_result != Action::Result::Success) {
            std::cout << "Arming failed" <<  std::endl;
        }

        // Set altitude to 3 meters
        action->set_takeoff_altitude(3.0);

        // Command Take off
        const Action::Result takeoff_result = action->takeoff();
        if (takeoff_result != Action::Result::Success) {
            std::cout << "Takeoff failed" << std::endl;
        }

    }

    void do_land()
    {
        const Action::Result landing_result = action->land();
        if (landing_result != Action::Result::Success) {
            std::cout << "Landing failed" << std::endl;
        }
    }

    void do_start_mission()
    {
        // Arm vehicle
        const Action::Result arm_result = action->arm();        
        if (arm_result != Action::Result::Success) {
            std::cout << "Arming failed" <<  std::endl;
        }

        // Start mission
        const Mission::Result result = mission->start_mission();
        if (result != Mission::Result::Success) {
            std::cout << "Mission start failed (" << std::endl;
        }
    }

    void do_return_to_launch()
    {
        const Action::Result rtl_result = action->return_to_launch();
        if (rtl_result != Action::Result::Success) {
            std::cout << "RTL failed (" << std::endl;
            return;
        }
    }

    void do_hold()
    {
        const Mission::Result result = mission->pause_mission();
        if (result != Mission::Result::Success) {
            std::cout << "Mission pause failed (" << std::endl;
        }
    }

    void do_continue_mission()
    {
        const Mission::Result result = mission->start_mission();
        if (result != Mission::Result::Success) {
            std::cout << "Mission continue failed (" << std::endl;
        }
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
