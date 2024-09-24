#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <tier4_perception_msgs/msg/traffic_light_element.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <tier4_perception_msgs/msg/traffic_signal.hpp>
#include <tier4_perception_msgs/msg/traffic_signal_array.hpp>



using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public:
    PlaybackNode(const std::string & args)
    : Node("roi_playback_node")
    {
    
    std::ignore = args;
	  roi_pub_ = this->create_publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>("/perception/traffic_light_recognition/rois", 1);

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

        if (serialized_message->topic_name != "/perception/traffic_light_recognition/traffic_light/detection/rois") {
          continue;
        }
    
        tier4_perception_msgs::msg::TrafficLightRoiArray roi_image_msg;
        serialization_.deserialize_message(&extracted_serialized_msg, &roi_image_msg);

        roi_pub_->publish(roi_image_msg);

        break;
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr roi_pub_;
    rclcpp::Serialization<tier4_perception_msgs::msg::TrafficLightRoiArray> serialization_;
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

