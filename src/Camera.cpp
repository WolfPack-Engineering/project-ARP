#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class Camera : public rclcpp::Node
{
public:
	explicit Camera() : Node("Camera")
	{
		camera_info_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
			"camera_info", 10, std::bind(&Camera::topic_callback, this, std::placeholders::_1));
	}

	rclcpp::Subscription<>
}