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
     

      loop_cap();
    }



  private:
    rclcpp::TimerBase::SharedPtr timer_;
   

    std::shared_ptr<rclcpp::Publisher<adp_core_msgs::msg::ObjectArrayStamped>> publisher_;
    size_t count_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> pub_jpg;

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
    cv::VideoCapture *cap;

    cv::Mat getNextFrame()
    {
      cv::Mat frame;
      *cap>>frame;

      return frame;	

    }

    void InitCap(){
      cap=new cv::VideoCapture("/home/fze2/Desktop/mot_files/test_data/aufnahme_Fahrt_Aachen.h264");
      if(!cap->isOpened()){
        throw std::runtime_error("VideoCap broken!"); 
      }
      cout<<"hi";

    }

    void loop_cap(){
      cv::Mat currentFrame;

      InitCap();
      while(!(currentFrame=getNextFrame()).empty()){
         //Führt zu seiner Exception. Nicht der gewünschte Weg aber das gewünschte Ergebnis.
        if(!rclcpp::ok()){
            cout<<"breche ab";
            break;
        }
        
        sensor_msgs::msg::CompressedImage image;  
        int colorReadCode=1;
        std_msgs::msg::Header myHeader;
        cv_bridge::CvImage myImage(myHeader,"bgr8",currentFrame);
        myImage.toCompressedImageMsg(image);

      
        image.header.stamp = this->now();
        image.header.frame_id = "video";

        pub_jpg->publish(image);
        RCLCPP_INFO(get_logger(), "Publishing and Doing shit.!");
        std::this_thread::sleep_for(0.1s);
      }
    }
    void loop()
    {
      
      auto mapWithFileNames=getFilePathes("/home/fze2/Desktop/kitti_data/testing/image_02/0000/");
		  for(auto i2:mapWithFileNames){
        
        //Führt zu seiner Exception. Nicht der gewünschte Weg aber das gewünschte Ergebnis.
        if(!rclcpp::ok()){
            cout<<"breche ab";
            break;
        }
        cv::Mat currentFrame;
        sensor_msgs::msg::CompressedImage image;  
        int colorReadCode=1;
        currentFrame = cv::imread("/home/fze2/Desktop/target_89-1.png", colorReadCode);
        std_msgs::msg::Header myHeader;
        cv_bridge::CvImage myImage(myHeader,"bgr8",currentFrame);
        myImage.toCompressedImageMsg(image);

      
        image.header.stamp = this->now();
        image.header.frame_id = "video";

        pub_jpg->publish(image);
        RCLCPP_INFO(get_logger(), "Publishing and Waiting for 1sec now.!");
        std::this_thread::sleep_for(1s);

      }
    }


    // adp_core_msgs::msg::ObjectArrayStamped getADP(){

    // }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}


