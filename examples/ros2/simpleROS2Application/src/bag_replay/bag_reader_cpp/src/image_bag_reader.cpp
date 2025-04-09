#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.h>

#include <application_data_object_management_msgs/msg/virtual_object.hpp>

#include <adom/application_interface.hpp>
#include <adom/parameters.h>


using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public:
    PlaybackNode(const std::string & args)
    : Node("image_playback_node")
    {    
    
    std::ignore = args;
	  image_pub_ = image_transport::create_publisher(this, "/sensing/camera/traffic_light/image_raw", rclcpp::QoS{1}.get_rmw_qos_profile()); 

    timer_ = this->create_wall_timer(
      100ms, std::bind(&PlaybackNode::timer_callback, this));

     reader_.open("rosbags/subset_image_roi");  
    }

  private:

    void timer_callback()
    {
      while (reader_.has_next()) {
        auto serialized_message = reader_.read_next();
        rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message->serialized_data);

        if (serialized_message->topic_name != "/sensing/camera/traffic_light/image_raw") {
          continue;
        }
    
        sensor_msgs::msg::Image input_image_msg;
        serialization_.deserialize_message(&extracted_serialized_msg, &input_image_msg);

        cv_bridge::CvImagePtr cv_ptr;
		    try {
          cv_ptr = cv_bridge::toCvCopy(input_image_msg, input_image_msg.encoding);
        } catch (cv_bridge::Exception & e) {
          RCLCPP_ERROR(
            get_logger(), "Could not convert from '%s' to 'bgr8'.", input_image_msg.encoding.c_str());
        }

        image_pub_.publish(cv_ptr->toImageMsg());

        break;
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher image_pub_;
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_;
    rosbag2_cpp::Reader reader_;
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag>" << std::endl;
    return 1;
  }


  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaybackNode>(argv[1]));
  rclcpp::shutdown();

  return 0;
}