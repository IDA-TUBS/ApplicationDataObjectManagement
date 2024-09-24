#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>


#include <application_data_object_management_msgs/msg/virtual_object.hpp>

#include <adom/application_interface.hpp>
#include <adom/parameters.h>
#include <adom/opencv_helper.h>
#include <adom/log.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class RoiReader : public rclcpp::Node
{
public:
  RoiReader()
      : Node("simple_roi_reader_adom")
  {

    // setup parameter struct for this home directory
    EntityDescriptor home_dir_descriptor;
    home_dir_descriptor.entity_id = 5;
    home_dir_descriptor.ingress_port = req_dir_ingress_port;
    strncpy(home_dir_descriptor.ip_address, subscriber_ip, sizeof(subscriber_ip));

    // create and initialize application interface
    adom_interface_ = std::make_unique<ApplicationInterface>(home_dir_descriptor);
    adom_interface_->initialize();

    // create data_structure for object IMAGE, this data_structure is required by the underlying layers to properly parse and deparse object samples
    Structure data_structure(TWO_DIMENSIONAL, FULL_HD_BLOCK_ROWS, FULL_HD_BLOCKS_IN_ROW, FULL_HD_V_PIXEL_PER_IMAGE, FULL_HD_H_PIXEL_PER_IMAGE, CHANNEL_NUMBER);

    // register IMAGE topic, so that the directory starts managing associated data samples
    adom_interface_->registerNewTopic(IMAGE, data_structure);


    // Initialize subscribers
    RCLCPP_INFO(this->get_logger(), "Initialize subscribers");
    virtual_object_sub_ = std::make_shared<message_filters::Subscriber<application_data_object_management_msgs::msg::VirtualObject>>(this, "/adom/virtual_object/image");
    roi_sub_ = std::make_shared<message_filters::Subscriber<tier4_perception_msgs::msg::TrafficLightRoiArray>>(this, "/perception/traffic_light_recognition/rois");

    // Initialize synchronizer
    sync_ = std::make_shared<message_filters::TimeSynchronizer<application_data_object_management_msgs::msg::VirtualObject, tier4_perception_msgs::msg::TrafficLightRoiArray>>(*virtual_object_sub_, *roi_sub_, 10);

    sync_->registerCallback(std::bind(&RoiReader::callback, this, _1, _2));

    // Initialize publisher
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("visualization_cached_image", 10);
  }

private:
  void callback(const application_data_object_management_msgs::msg::VirtualObject::ConstSharedPtr virtual_object_msg, const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr roi_msg)
  {
    if (roi_msg->rois.empty()) {
      RCLCPP_WARN(this->get_logger(), "%s", "No ROI Information received.");
      return;
    }

    // Extract all information from virtual object announcement
    cv::Mat cached_image = cv::Mat::zeros(virtual_object_msg->object_structure.object_height, virtual_object_msg->object_structure.object_width, CV_8UC3);
    StructureType type = static_cast<StructureType>(virtual_object_msg->object_structure.structure_type);
    Structure data_structure(type, 
                              virtual_object_msg->object_structure.block_rows, 
                              virtual_object_msg->object_structure.block_cols, 
                              virtual_object_msg->object_structure.object_height, 
                              virtual_object_msg->object_structure.object_width, 
                              virtual_object_msg->object_structure.object_channels);
    
    // Define the list of block areas to read
    std::vector<std::vector<uint16_t>> block_lists;

    // Collect Info on required block per Roi
    for (const auto &roi : roi_msg->rois)
    {
      // translate roi to block access
      block_lists.push_back(findBlocksForReadAccess(roi.roi.x_offset, roi.roi.y_offset, roi.roi.x_offset + roi.roi.width, roi.roi.y_offset + roi.roi.height, data_structure));
    }

    // request blocks
    std::vector<std::future<int>> futures;
    for (const auto& blocks : block_lists) {
        futures.push_back(adom_interface_->readPartialData(IMAGE, blocks, cached_image.data));
    }

    // wait for all blocks to arrive
    bool all_success = true; // Variable to check whether all requests were successful
    for (auto& future : futures) {
        int result = future.get();
        if (result != 0) {
            all_success = false;
        }
    }

    if (all_success) {
      // Convert ROS image message to OpenCV image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage());
      cv_ptr->encoding = "bgr8";      
      cv_ptr->header.stamp = this->now();
      try
      {
        // convert return from adom to cv_ptr
        cached_image.copyTo(cv_ptr->image);
      }
      catch (cv_bridge::Exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      // Draw rectangles around the RoIs
      for (const auto &roi : roi_msg->rois)
      {
        cv::Rect rect(roi.roi.x_offset, roi.roi.y_offset, roi.roi.width, roi.roi.height);
        cv::rectangle(cv_ptr->image, rect, cv::Scalar(0, 255, 0), 2);
      }

      // Save the image as a PNG file
      std::string filename = "published_cached_image_" + std::to_string(virtual_object_msg->object_sequence_nr) + ".png"; 
      cv::imwrite(filename, cv_ptr->image);

      // Convert OpenCV image back to ROS image message
      auto output_msg = cv_ptr->toImageMsg();
      output_msg->header.stamp = this->now();
      output_msg->header.frame_id = "cached_image";

      // Publish the visualized image
      image_pub_->publish(*output_msg);
      RCLCPP_INFO(this->get_logger(), "%s", "Published cached image");

    } else {        
        RCLCPP_INFO(this->get_logger(), "Something went wrong");
    }

    
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  std::shared_ptr<message_filters::Subscriber<application_data_object_management_msgs::msg::VirtualObject>> virtual_object_sub_;
  std::shared_ptr<message_filters::Subscriber<tier4_perception_msgs::msg::TrafficLightRoiArray>> roi_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<application_data_object_management_msgs::msg::VirtualObject, tier4_perception_msgs::msg::TrafficLightRoiArray>> sync_;
  std::unique_ptr<ApplicationInterface> adom_interface_;
};

int main(int argc, char *argv[])
{
      
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoiReader>());
  rclcpp::shutdown();
  return 0;
}
