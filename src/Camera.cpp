#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class Camera : public rclcpp::Node
{
public:
	explicit Camera() : Node("Camera")
	{
		camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
			"camera_info", 10, std::bind(&Camera::topic_callback, this, std::placeholders::_1));
		camera_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
			"image", 10, std::bind(&Camera::image_callback, this, std::placeholders::_1));
		
	}

private:

	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {}

	void topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) const {}	

	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_sub_;
	// rclcpp::Publisher<>::SharedPtr cones_location;
}