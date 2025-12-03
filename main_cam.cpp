#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include<arpa/inet.h>
#include<unistd.h>

#include "chad_mod.h"

int main() {
    std::string input_pipeline = "udpsrc port=5600 ! "
                                 "application/x-rtp, payload=96 ! "
                                 "rtph264depay ! h264parse ! avdec_h264 ! "
                                 "videoconvert ! appsink sync=false";

    // OUTPUT: Send processed video to MediaMTX (so it shows in Cockpit)
    // NOTE: '192.168.2.2' is the BlueOS IP. 
    std::string output_url = "rtsp://192.168.2.2:8554/processed_video";
    std::string writer_pipeline = "appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! rtspclientsink location=" + output_url;

    // Open Input
    cv::VideoCapture cap(input_pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cout << "Impossible d'ouvrir la video" << std::endl;
        return -1;
    }

    float resize_factor = 0.5f;

    // Variables du PID
    float KPX = 1;
    float KIX = 1;
    float KDX = 1;
    float KPY = 1;
    float KIY = 1;
    float KDY = 1;
    bool pid_x_enabled = true;
    bool pid_y_enabled = true;

    // Création du socket pour l'envoie
    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr;
    addr.sin_family = AF_INET;                                                          //IPV4
    addr.sin_port = htons(6969);                                                        //Port de destination
    inet_pton(AF_INET, "10.182.245.67", &addr.sin_addr);                             //Adresse IP
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

    Tracker tracker(1.6f, 12, 0.02f, 1000, 3, 15);

    cv::Mat frame, frame0, gray0;
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

    
    while (true) {
        bool ret = cap.read(frame);
        if (!ret || frame.empty())
            break;

        // resize pour gagner en rapidité
        cv::Mat frame_small;
        cv::resize(frame, frame_small, cv::Size(), resize_factor, resize_factor);

        if (ref_set) {
            // calcul du déplacement
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

            // dessin de la flèche sur la frame
            cv::arrowedLine(frame,
                            cv::Point(cy, cz),
                            cv::Point(cy + static_cast<int>(Yf),
                                      cz + static_cast<int>(Zf)),
                            cv::Scalar(0, 0, 255),
                            3,
                            cv::LINE_AA,
                            0,
                            0.2);

            // écriture des informations sur la frame
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

        // affichage de la frame
        cv::imshow("SIFT Poursuite", frame);

        // 'r' prend une nouvelle référence et 'q' termine le programme
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