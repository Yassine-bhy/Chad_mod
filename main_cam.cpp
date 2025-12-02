#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "chad_mod.h"

int main() {
    cap = cv.VideoCapture(0, cv.CAP_DSHOW)
    if (!cap.isOpened()) {
        std::cout << "Impossible d'ouvrir la video" << std::endl;
        return -1;
    }

    float resize_factor = 0.5f;

    float KPX = 1;
    float KIX = 1;
    float KDX = 1;
    float KPY = 1;
    float KIY = 1;
    float KDY = 1;
    bool pid_x_enabled = true;
    bool pid_y_enabled = true;

    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr;
    addr.sin_family = AF_INET;                                                          //IPV4
    addr.sin_port = htons(9000);                                                        //Port de destination
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);                               //Adresse IP
    struct Packet { float KPX;
                    float KIX;
                    float KDX;
                    float KPY;
                    float KIY;
                    float KDY;
                    float X;
                    float Y;
                    float Z;
                    bool pid_x_enabled = true;
                    bool pid_y_enabled = true;
                    int nb_kp_ref;
                    int nb_kp_cur;
                    int nb_good;
            };

    Tracker tracker(1.6f, 10, 0.02f, 1000, 3, 15);

    cv::Mat frame, frame0, gray0;
    std::vector<cv::KeyPoint> kp0;
    cv::Mat des0;
    std::vector<uint8_t> Rchan0;

    std::vector<cv::KeyPoint> kp2;
    cv::Mat des2, gray2;
    std::vector<uint8_t> Rchan2;

    bool ref_set = false;
    float Xf = 0.0f, Yf = 0.0f, Zf = 0.0f;
    float alpha = 0.2f;

    std::vector<float> rapports;

    while (true) {
        bool ret = cap.read(frame);
        if (!ret || frame.empty())
            break;

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

            // Envoie UDP
            Packet p;
            p.KPX = KPX;
            p.KIX = KIX;
            p.KDX = KDY;
            p.KPY = KPY;
            p.KIY = KIY;
            p.KDY = KDY;
            p.X = X;
            p.Y = Y;
            p.Z = Z;
            p.pid_x_enabled = pid_x_enabled;
            p.pid_y_enabled = pid_y_enabled;
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

        char key = static_cast<char>(cv::waitKey(delay));
        if (key == 'q')
            break;

        if (key == 'r') {
            frame0 = frame_small.clone();
            tracker.computeReference(frame0, kp0, des0, gray0, Rchan0);
            ref_set = true;
            Xf = Yf = Zf = 0.0f;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}