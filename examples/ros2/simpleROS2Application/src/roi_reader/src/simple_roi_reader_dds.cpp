#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class RoiReader : public rclcpp::Node
{
public:
  RoiReader()
      : Node("simple_roi_reader_dds")
  {
    // Initialize subscribers
    image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/sensing/camera/traffic_light/image_raw");
    roi_sub_ = std::make_shared<message_filters::Subscriber<tier4_perception_msgs::msg::TrafficLightRoiArray>>(this, "/perception/traffic_light_recognition/rois");

    // Initialize synchronizer
    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, tier4_perception_msgs::msg::TrafficLightRoiArray>>(
        *image_sub_, *roi_sub_, 10);

    sync_->registerCallback(std::bind(&RoiReader::callback, this, _1, _2));

    // Initialize publisher
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("visualization_image", 10);
  }

private:
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr roi_msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
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

    // Convert OpenCV image back to ROS image message
    auto output_msg = cv_ptr->toImageMsg();
    output_msg->header = image_msg->header;

    // Publish the visualized image
    image_pub_->publish(*output_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
  std::shared_ptr<message_filters::Subscriber<tier4_perception_msgs::msg::TrafficLightRoiArray>> roi_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, tier4_perception_msgs::msg::TrafficLightRoiArray>> sync_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoiReader>());
  rclcpp::shutdown();
  return 0;
}
