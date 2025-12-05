#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class BlueRovVision : public rclcpp::Node {
public:
    BlueRovVision() : Node("bluerov_vision_node") {
        std::string topic_name = "/bluerov/sensors/CamL/image_color";
        rclcpp::QoS qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name,
            qos_policy,
            std::bind(&BlueRovVision::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Waiting for video frames on: %s", topic_name.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            cv::circle(frame, cv::Point(frame.cols/2, frame.rows/2), 50, cv::Scalar(0, 255, 0), 3);

            cv::imshow("BlueROV CamL Stream", frame);
            cv::waitKey(1); 

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlueRovVision>());
    rclcpp::shutdown();
    return 0;
}