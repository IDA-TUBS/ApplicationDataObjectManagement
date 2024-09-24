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
#include <adom/log.hpp>


using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public:
    PlaybackNode(const std::string & args)
    : Node("image_playback_node"),
      sequence_number_(0) 
    {

    // setup parameter struct for this home directory
    EntityDescriptor home_dir_descriptor;
    home_dir_descriptor.entity_id = 0;
    home_dir_descriptor.ingress_port = home_dir_ingress_port;
    strncpy(home_dir_descriptor.ip_address, publisher_ip, sizeof(publisher_ip));

    // create and initialize application interface
    adom_interface_ = std::make_unique<ApplicationInterface>(home_dir_descriptor);
    adom_interface_->initialize();

    // create data_structure for object IMAGE, this data_structure is required by the underlying layers to properly parse and deparse object samples
    Structure data_structure(TWO_DIMENSIONAL, 54, 96, 1080, 1920, 3);
    
    // register IMAGE topic, so that the directory starts managing associated data samples
    adom_interface_->registerNewTopic(IMAGE, data_structure);


    std::ignore = args;
	  image_pub_ = image_transport::create_publisher(this, "/sensing/camera/traffic_light/image_raw", rclcpp::QoS{1}.get_rmw_qos_profile()); 

    // Add publisher for virtual object
	  virtual_object_pub_ = this->create_publisher<application_data_object_management_msgs::msg::VirtualObject>("/adom/virtual_object/image", 1);

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

        // prepare virtual object
        sequence_number_++;

        auto msg = application_data_object_management_msgs::msg::VirtualObject();
        auto object_structure = application_data_object_management_msgs::msg::ObjectStructure();
        object_structure.block_rows = 54;           
        object_structure.block_cols = 96;    
        object_structure.object_height = 1080;         
        object_structure.object_width = 1920;          
        object_structure.object_channels = CHANNEL_NUMBER;       
        object_structure.structure_type = application_data_object_management_msgs::msg::ObjectStructure::TWO_DIMENSIONAL; 

        msg.object_structure = object_structure;
        msg.object_type = "IMAGE";  
        msg.object_sequence_nr = sequence_number_; 
        msg.header.stamp = input_image_msg.header.stamp;

        virtual_object_pub_->publish(msg);

        // forward data to interface for directory to manage
        adom_interface_->registerNewData(IMAGE, cv_ptr->image.data);

        auto image_msg = cv_ptr->toImageMsg();
        image_msg->header.frame_id = std::to_string(sequence_number_);
        
        image_pub_.publish(image_msg);

        break;
      }
    }
    

    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher image_pub_;
    rclcpp::Publisher<application_data_object_management_msgs::msg::VirtualObject>::SharedPtr virtual_object_pub_;
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_;
    rosbag2_cpp::Reader reader_;
    uint16_t sequence_number_; 
    std::unique_ptr<ApplicationInterface> adom_interface_;
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