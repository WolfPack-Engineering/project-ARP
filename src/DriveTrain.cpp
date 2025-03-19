#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <airsim_interfaces/msg/car_controls.hpp>

class DriveTrain : public rclcpp::Node
{
public:
    explicit DriveTrain() : Node("DriveTrain")
    {
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "Car_Commands", 10, std::bind(&DriveTrain::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<airsim_interfaces::msg::CarControls>("/airsim_node/RaceCar/car_cmd", 10);
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received CarCommands message: linear.x: '%f', angular.z: '%f'", msg.linear.x, msg.angular.z);
        publish_cmds(msg);
    }

    void publish_cmds(const geometry_msgs::msg::Twist cmd) const
    {
        auto msg = airsim_interfaces::msg::CarControls();
        msg.manual = true;
        msg.gear_immediate = true;
        msg.manual_gear = 1;
        msg.throttle = cmd.linear.x;
        msg.steering = cmd.angular.z;
        if (cmd.linear.x == 0.0) {
            msg.brake = true;
        }
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published CarControls message: throttle: '%f', steering: '%f'", msg.throttle, msg.steering);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Publisher<airsim_interfaces::msg::CarControls>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveTrain>());
    rclcpp::shutdown();
    return 0;
}