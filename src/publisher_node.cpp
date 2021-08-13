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

#include <thread>



using namespace std::chrono_literals;
using namespace std;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */



class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      
      publisher_ = this->create_publisher<adp_core_msgs::msg::ObjectArrayStamped>("meineStrings", 10);
      pub_jpg = this->create_publisher<sensor_msgs::msg::CompressedImage>("meineBilder", 100);
      loop();
    }



  private:
    rclcpp::TimerBase::SharedPtr timer_;
   

    std::shared_ptr<rclcpp::Publisher<adp_core_msgs::msg::ObjectArrayStamped>> publisher_;
    size_t count_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> pub_jpg;
    

    // void timer_callback_temp()
    // {
      
    //   adp_core_msgs::msg::ObjectArrayStamped message;
    //   adp_core_msgs::msg::ObjectStamped objectStamped;
    //   int numberOfObjects=0;

    //   //ObjectStamped
    //   int sensorIndex=1;
    //   int objectIndex=1;
    //   int motion_model=1;
    //   adp_core_msgs::msg::MeasurementVector measurementVector;
    //   measurementVector.x=0;
    //   measurementVector.y=0;
    //   measurementVector.z=0;

    //   measurementVector.vx=0;
    //   measurementVector.vy=0;
    //   measurementVector.vz=0;

    //   measurementVector.ax=1;
    //   measurementVector.ay=1;
    //   measurementVector.az=1;
    //   measurementVector.orientation=0;
      
    //   adp_core_msgs::msg::MeasurementNoise measurementNoise;

    //   measurementNoise.x_rms=1;
    //   measurementNoise.y_rms=1;
    //   measurementNoise.z_rms=1;
    //   measurementNoise.vx_rms=1;
    //   measurementNoise.vy_rms=1;
    //   measurementNoise.vz_rms=1;
    //   measurementNoise.ax_rms=1;
    //   measurementNoise.ay_rms=1;
    //   measurementNoise.az_rms=1;
    //   measurementNoise.orientation_rms=1;



    //   adp_core_msgs::msg::MeasurementParameters measurementParameters;
    //   measurementParameters.origin_x=1;
    //   measurementParameters.origin_y=1;
    //   measurementParameters.origin_z=1;
    //   measurementParameters.orientation_x=1;
    //   measurementParameters.orientation_y=1;
    //   measurementParameters.orientation_z=1;
    //   measurementParameters.frame=1;


    //   adp_core_msgs::msg::ObjectAttributes objectAttributes;
    //   objectAttributes.dynamic_type=1;
    //   objectAttributes.object_class=1;
    //   objectAttributes.object_class_confidence=1;
    //   objectAttributes.object_length=1;
    //   objectAttributes.object_width=1;

      
    //   objectStamped.measurement_noise=measurementNoise;
    //   objectStamped.measurement_parameters=measurementParameters;
    //   objectStamped.measurement_vector=measurementVector;
    //   objectStamped.object_attributes=objectAttributes;

    //   message.objects[0]=objectStamped;
      
     
    //   RCLCPP_INFO(this->get_logger(), "I am Logging");
    //   publisher_->publish(message);
    // }

     map<int, string> getFilePathes(string directory)
    {
        map<int, string> map1;
        int number_of_zeros=6;
    
      string original_string=to_string(42);
      std::string dest = std::string( number_of_zeros, '0').append( original_string);

      for(int i=0; i<1000; i++){
            
        string original_string=to_string(i);
        std::string dest = std::string( number_of_zeros-original_string.size(), '0').append( original_string);


            string filepath=directory+dest+".png";

            if (FILE *file = fopen(filepath.c_str(), "r")) 
            {
                fclose(file);
                map1.insert(std::pair<int, string>(i,filepath));
                
            }else {break;}
            
            
      }

        return map1;
    }
    void loop()
    {
      
      auto mapWithFileNames=getFilePathes("/home/fze2/Desktop/kitti_data/testing/image_02/0000/");
		  for(auto i2:mapWithFileNames){
        cv::Mat currentFrame;
        sensor_msgs::msg::CompressedImage img_msg_compressed;  
        int colorReadCode=1;
        currentFrame = cv::imread(i2.second, colorReadCode);
        std_msgs::msg::Header myHeader;
        cv_bridge::CvImage myImage(myHeader,"bgr8",currentFrame);
        myImage.toCompressedImageMsg(img_msg_compressed);
        pub_jpg->publish(img_msg_compressed);
        RCLCPP_INFO(get_logger(), "Publishing and Waiting for 1sec now.!");
        std::this_thread::sleep_for(1s);

      }
    }


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


