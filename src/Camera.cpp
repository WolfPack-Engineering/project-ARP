#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/msg/twist.hpp>

static const std::string FILTER_WINDOW = "Filtered Image";
static const std::string OPENCV_WINDOW = "Image window";
static const std::string EDGE_WINDOW = "Edge window";

int hmin_blue, smin_blue, vmin_blue, hmin_yellow, smin_yellow, vmin_yellow;
int hmax_blue, smax_blue, vmax_blue, hmax_yellow, smax_yellow, vmax_yellow;

int erode1, erode2, dilate;

class Camera : public rclcpp::Node
{
public:
	explicit Camera() : Node("Camera")
	{
		camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
			"/airsim_node/RaceCar/front/Scene/camera_info", 10, std::bind(&Camera::topic_callback, this, std::placeholders::_1));
		camera_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/airsim_node/RaceCar/front/Scene", 10, std::bind(&Camera::image_callback, this, std::placeholders::_1));
		heading_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("Car_Commands", 10);
		
		cv::namedWindow(OPENCV_WINDOW);
		cv::namedWindow(FILTER_WINDOW);
		cv::namedWindow(EDGE_WINDOW);

		//hmin_blue = smin_blue = vmin_blue = hmin_yellow = smin_yellow = vmin_yellow = 0;
		//hmax_blue = hmax_yellow = 179;
		//smax_blue = vmax_blue = smax_yellow = vmax_yellow = 255;

		hmin_blue = 97, hmax_blue = 103;
		smin_blue = 38, smax_blue = 255;
		vmin_blue = 115, vmax_blue = 250;

		hmin_yellow = 22, hmax_yellow = 62;
		smin_yellow = 8, smax_yellow = 235;
		vmin_yellow = 61, vmax_yellow = 250;

		erode1 = 1, erode2 = 6, dilate = 7;
	}

	~Camera()
	{
		cv::destroyAllWindows();
	}

private:

	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const 
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		}
		catch(cv_bridge::Exception& e)
		{
			RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		}

		cv::Mat cropped_img = cv_ptr->image(cv::Rect(0, 350, cv_ptr->image.size().width, cv_ptr->image.size().height - 350));

		cv::createTrackbar("H Min Blue", FILTER_WINDOW, &hmin_blue, 179, nullptr);
		cv::createTrackbar("S Min Blue", FILTER_WINDOW, &smin_blue, 255, nullptr);
		cv::createTrackbar("V Min Blue", FILTER_WINDOW, &vmin_blue, 255, nullptr);
		cv::createTrackbar("H Max Blue", FILTER_WINDOW, &hmax_blue, 179, nullptr);
		cv::createTrackbar("S Max Blue", FILTER_WINDOW, &smax_blue, 255, nullptr);
		cv::createTrackbar("V Max Blue", FILTER_WINDOW, &vmax_blue, 255, nullptr);

		cv::createTrackbar("H Min Yellow", FILTER_WINDOW, &hmin_yellow, 179, nullptr);
		cv::createTrackbar("S Min Yellow", FILTER_WINDOW, &smin_yellow, 255, nullptr);
		cv::createTrackbar("V Min Yellow", FILTER_WINDOW, &vmin_yellow, 255, nullptr);
		cv::createTrackbar("H Max Yellow", FILTER_WINDOW, &hmax_yellow, 179, nullptr);
		cv::createTrackbar("S Max Yellow", FILTER_WINDOW, &smax_yellow, 255, nullptr);
		cv::createTrackbar("V Max Yellow", FILTER_WINDOW, &vmax_yellow, 255, nullptr);
		

		cv::Mat hsv_img;
		cv::cvtColor(cropped_img, hsv_img, cv::COLOR_BGR2HSV);

		// Define color ranges for the cones
		cv::Scalar lower_blue(hmin_blue, smin_blue, vmin_blue), lower_yellow(hmin_yellow, smin_yellow, vmin_yellow);
		cv::Scalar upper_blue(hmax_blue, smax_blue, vmax_blue), upper_yellow(hmax_yellow, smax_yellow, vmax_yellow);

		cv::Mat blue_img, yellow_img;
		cv::inRange(hsv_img, lower_blue, upper_blue, blue_img);
		cv::inRange(hsv_img, lower_yellow, upper_yellow, yellow_img);

		cv::Mat filtered_img;
		cv::bitwise_or(blue_img, yellow_img, filtered_img);

		cv::createTrackbar("Erode_1", EDGE_WINDOW, &erode1, 20, nullptr);
		cv::createTrackbar("Erode_2", EDGE_WINDOW, &erode2, 20, nullptr);
		cv::createTrackbar("Dilate", EDGE_WINDOW, &dilate, 20, nullptr);

		cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
		cv::Mat smoother_blue, smoother_yellow;
		cv::erode(blue_img, smoother_blue, kernal, cv::Point(-1, -1), erode1);
		cv::dilate(smoother_blue, smoother_blue, kernal, cv::Point(-1,-1), dilate);
		cv::erode(smoother_blue, smoother_blue, kernal, cv::Point(-1, -1), erode2);

		cv::erode(yellow_img, smoother_yellow, kernal, cv::Point(-1, -1), erode1);
		cv::dilate(smoother_yellow, smoother_yellow, kernal, cv::Point(-1,-1), dilate);
		cv::erode(smoother_yellow, smoother_yellow, kernal, cv::Point(-1, -1), erode2);
	
		cv::Mat edges_blue, edges_yellow, edges_img;
		cv::Canny(smoother_blue, edges_blue, 100, 200);
		cv::Canny(smoother_yellow, edges_yellow, 100, 200);
		cv::bitwise_or(edges_blue, edges_yellow, edges_img);

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(edges_img, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		std::vector<std::pair<cv::Point, int>> cones;

		for (const auto& cnt : contours) {
			std::vector<cv::Point> approx;
			cv::approxPolyDP(cnt, approx, 0.06 * cv::arcLength(cnt, true), true);

			if (approx.size() == 3 && cv::contourArea(cnt) < 3000 && cv::contourArea(cnt) > 100) {
				cv::Rect boundingRect = cv::boundingRect(approx);
				cv::rectangle(cropped_img, boundingRect, cv::Scalar(0, 255, 0), 3);
				cv::putText(cropped_img, "traffic cone; area: " + std::to_string(static_cast<int>(cv::contourArea(cnt))),
					cv::Point(boundingRect.x, boundingRect.y),
					cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
				
				cv::Moments M = cv::moments(cnt);
				cv::Point coa = cv::Point(M.m10 / M.m00, M.m01 / M.m00);

				cv::circle(cropped_img, coa, 7, cv::Scalar(255, 0, 0), -1);

				cones.push_back({coa, static_cast<int>(cv::contourArea(cnt))});
			}
		}

		if (!cones.empty()) {
			
			int x_avg = 0, y_avg = 0, weight = 0, i = 0;
			for (const auto& pair : cones) {
				if (pair.second < 3000 && pair.second > 100) { weight += pair.second; }
			}

			std::cout << weight << std::endl;

			for (const auto& pair : cones) {
				if (pair.second < 3000 && pair.second > 100) {			
					x_avg += pair.first.x;
					y_avg += pair.first.y;
					i++;
				}
			}

			x_avg /= i;
			y_avg /= i;

			cv::circle(cropped_img, cv::Point(x_avg, y_avg), 7, cv::Scalar(0, 0, 255), -1);

			float heading = 2 * static_cast<float>(x_avg - cropped_img.size().width / 2) / cropped_img.size().width;

			if (heading == 0.5) { heading = 0; }

			auto cmd = geometry_msgs::msg::Twist();
			cmd.linear.x = 0.3;
			cmd.angular.z = heading;
			heading_pub_->publish(cmd);
			RCLCPP_INFO(this->get_logger(), "linear.x: '%f', angular.z: '%f', x_avg: '%i'", cmd.linear.x, cmd.angular.z, x_avg);
		} else {
			auto cmd = geometry_msgs::msg::Twist();
			cmd.linear.x = 0.0;
			cmd.angular.z = 0.0;
			heading_pub_->publish(cmd);
			RCLCPP_INFO(this->get_logger(), "Braking!!!");
		}

		cv::imshow(EDGE_WINDOW, edges_img);

		//Update GUI Window
		cv::imshow(OPENCV_WINDOW, cropped_img);
		cv::imshow(FILTER_WINDOW, filtered_img);
		cv::waitKey(3);
		
	}

	void topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) const {
		msg.get();
	}	

	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_sub_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr heading_pub_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<Camera>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
