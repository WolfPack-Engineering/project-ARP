#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string FILTER_WINDOW = "Filtered Image";
static const std::string OPENCV_WINDOW = "Image window";
static const std::string EDGE_WINDOW = "Edge window";

int hmin_blue, smin_blue, vmin_blue, hmin_yellow, smin_yellow, vmin_yellow;
int hmax_blue, smax_blue, vmax_blue, hmax_yellow, smax_yellow, vmax_yellow;

class Camera : public rclcpp::Node
{
public:
	explicit Camera() : Node("Camera")
	{
		camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
			"/airsim_node/RaceCar/front/Scene/camera_info", 10, std::bind(&Camera::topic_callback, this, std::placeholders::_1));
		camera_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/airsim_node/RaceCar/front/Scene", 10, std::bind(&Camera::image_callback, this, std::placeholders::_1));
		
		cv::namedWindow(OPENCV_WINDOW);
		cv::namedWindow(FILTER_WINDOW);
		cv::namedWindow(EDGE_WINDOW);

		hmin_blue = smin_blue = vmin_blue = hmin_yellow = smin_yellow = vmin_yellow = 0;
		hmax_blue = hmax_yellow = 179;
		smax_blue = vmax_blue = smax_yellow = vmax_yellow = 255;

		hmin_blue = 97, hmax_blue = 103;
		smin_blue = 38, smax_blue = 255;
		vmin_blue = 115, vmax_blue = 250;

		hmin_yellow = 22, hmax_yellow = 50;
		smin_yellow = 14, smax_yellow = 207;
		vmin_yellow = 92, vmax_yellow = 250;
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
		cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);

		cv::Mat cropped_img = hsv_img(cv::Rect(0, 350, hsv_img.size().width, hsv_img.size().height - 350));

		// Define color ranges for the cones
		cv::Scalar lower_blue(hmin_blue, smin_blue, vmin_blue), lower_yellow(hmin_yellow, smin_yellow, vmin_yellow);
		cv::Scalar upper_blue(hmax_blue, smax_blue, vmax_blue), upper_yellow(hmax_yellow, smax_yellow, vmax_yellow);

		cv::Mat blue_img, yellow_img;
		cv::inRange(cropped_img, lower_blue, upper_blue, blue_img);
		cv::inRange(cropped_img, lower_yellow, upper_yellow, yellow_img);

		cv::Mat filtered_img;
		cv::bitwise_or(blue_img, yellow_img, filtered_img);

		cv::Mat edges_img;
		cv::Canny(filtered_img, edges_img, 100, 200);
		//cv::imshow(EDGE_WINDOW, edges_img);

		cv::Mat kernal = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
		cv::Mat smoothed_img;
		cv::erode(filtered_img, smoothed_img, kernal, cv::Point(-1, -1), 1);
		cv::Mat smoother_img;
		cv::dilate(smoothed_img, smoother_img, kernal, cv::Point(-1,-1), 5);
		cv::erode(smoother_img, smoother_img, kernal, cv::Point(-1, -1), 4);

		

		//Update GUI Window
		cv::imshow(OPENCV_WINDOW, hsv_img);
		cv::imshow(FILTER_WINDOW, filtered_img);
		cv::imshow(EDGE_WINDOW, cv_ptr->image);
		cv::waitKey(3);
		
	}

	void topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) const 
	{
		msg.get();
	}	

	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_sub_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<Camera>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
