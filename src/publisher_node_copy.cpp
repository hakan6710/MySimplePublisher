#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

//So funktioniert include
#include "adp_core_msgs/msg/object_array_stamped.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/opencv.hpp>
// #include <sensor_msgs/msg/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
//#include "../include/cv_bridge_galactic.hpp"

using namespace std::chrono_literals;

using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */



class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      

      image_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>("meineBilderAlda", 10, std::bind(&MinimalPublisher::timer_callback, this, _1));
     
      
    }



  private:
    rclcpp::TimerBase::SharedPtr timer_;
   

  
    size_t count_;

    

    void timer_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const 
    {
      
      auto lol=cv_bridge::toCvCopy(msg);
      
      cv::Mat readFrame;
      readFrame=lol->image;

      cv::imshow("DeepSortTracking", readFrame);
			cv::waitKey(10);

      //(memcpy(&img_msg_compressed.data[0], currentFrame.data, compressed_size);
     
      RCLCPP_INFO(this->get_logger(), "lol ist rdy?");
    }
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage >::SharedPtr image_sub;  
    adp_core_msgs::msg::ObjectArrayStamped getADP(){
    


    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}


