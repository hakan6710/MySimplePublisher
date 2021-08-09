#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

//So funktioniert include
#include "adp_core_msgs_test/msg/objectstamped.hpp"

//funktioniert nicht, weil ObjectStamped
//#include "adp_core_msgs/msg/objectStamped.hpp"
using namespace std::chrono_literals;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      
      publisher_ = this->create_publisher<adp_core_msgs_test::msg::Objectstamped>("meineStringsAlda", 10);
      
    }



  private:
    rclcpp::TimerBase::SharedPtr timer_;
   

    std::shared_ptr<rclcpp::Publisher<adp_core_msgs_test::msg::Objectstamped>> publisher_;
    size_t count_;
  
    void timer_callback()
    {
      //auto test=adp_core_msgs::ObjectStamped();
      auto message= adp_core_msgs_test::msg::Objectstamped();
      message.motion_model=2;
      RCLCPP_INFO(this->get_logger(), "I am Logging");
      publisher_->publish(message);
    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}