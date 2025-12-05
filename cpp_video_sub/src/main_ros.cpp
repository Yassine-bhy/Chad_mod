#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <arpa/inet.h>
#include <unistd.h>

#include "chad_mod.h"
#include "packet.h"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bluerov_vision_node");

    float resize_factor = 0.5f;

    float KPX = 1;
    float KIX = 1;
    float KDX = 1;
    float KPY = 1;
    float KIY = 1;
    float KDY = 1;
    float KPZ = 1;
    float KIZ = 1;
    float KDZ = 1;
    bool pid_x_enabled = true;
    bool pid_y_enabled = true;
    bool pid_z_enabled = true;

    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(6969);
    inet_pton(AF_INET, "10.182.245.67", &addr.sin_addr);

    Tracker tracker(1.6f, 12, 0.02f, 0, 3, 10);

    cv::Mat frame0, gray0;
    std::vector<cv::KeyPoint> kp0;
    cv::Mat des0;
    std::vector<float> Rchan0;

    std::vector<cv::KeyPoint> kp2;
    cv::Mat des2, gray2;
    std::vector<float> Rchan2;

    bool ref_set = false;
    float Xf = 0.0f, Yf = 0.0f, Zf = 0.0f;
    float alpha = 0.2f;

    std::vector<float> rapports;

    auto callback = [&](const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (frame.empty()) return;

        cv::Mat frame_small;
        cv::resize(frame, frame_small, cv::Size(), resize_factor, resize_factor);

        if (ref_set) {
            Displacement d = tracker.computeDisplacement(
                kp0, des0, frame0, Rchan0,
                frame_small, kp2, des2, gray2, Rchan2,
                rapports
            );

            float X = d.X;
            float Y = d.Y * (1.0f / resize_factor);
            float Z = d.Z * (1.0f / resize_factor);

            Xf = alpha * X + (1.0f - alpha) * Xf;
            Yf = alpha * Y + (1.0f - alpha) * Yf;
            Zf = alpha * Z + (1.0f - alpha) * Zf;

            Packet p;
            p.KPX = KPX;
            p.KIX = KIX;
            p.KDX = KDX;
            p.KPY = KPY;
            p.KIY = KIY;
            p.KDY = KDY;
            p.KPZ = KPZ;
            p.KIZ = KIZ;
            p.KDZ = KDZ;
            p.X = X;
            p.Y = Y;
            p.Z = Z;
            p.pid_x_enabled = pid_x_enabled;
            p.pid_y_enabled = pid_y_enabled;
            p.pid_z_enabled = pid_z_enabled;
            p.nb_kp_ref = d.nb_kp_ref;
            p.nb_kp_cur = d.nb_kp_cur;
            p.nb_good = d.nb_good;

            sendto(udp_sock, &p, sizeof(Packet), 0, (sockaddr*)&addr, sizeof(addr));

            int cy = frame.cols / 2;
            int cz = frame.rows / 2;

            cv::arrowedLine(frame,
                            cv::Point(cy, cz),
                            cv::Point(cy + static_cast<int>(Yf),
                                      cz + static_cast<int>(Zf)),
                            cv::Scalar(0, 0, 255),
                            3,
                            cv::LINE_AA,
                            0,
                            0.2);

            std::string mouvement = (Xf > 0.0f) ? "avance" : "recule";

            std::string info = "dx=" + std::to_string(Xf) + " (" + mouvement + ")" +
                               ", dy=" + std::to_string(Yf) +
                               ", dz=" + std::to_string(Zf) +
                               ", kp_ref=" + std::to_string(d.nb_kp_ref) +
                               ", kp_cur=" + std::to_string(d.nb_kp_cur) +
                               ", good=" + std::to_string(d.nb_good);

            cv::putText(frame,
                        info,
                        cv::Point(10, 50),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(0, 0, 255),
                        2);
        }

        cv::imshow("SIFT Poursuite", frame);

        char key = static_cast<char>(cv::waitKey(1));
        if (key == 'q') {
            rclcpp::shutdown();
        }

        if (key == 'r') {
            frame0 = frame_small.clone();
            tracker.computeReference(frame0, kp0, des0, gray0, Rchan0);
            ref_set = true;
            Xf = Yf = Zf = 0.0f;
        }
    };

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/bluerov/sensors/CamL/image_color", 
        qos, 
        callback
    );

    std::cout << "Waiting for ROS 2 images..." << std::endl;

    rclcpp::spin(node);

    close(udp_sock);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}