#pragma once

#include <vector>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

struct Displacement {
    float X;
    float Y;
    float Z;
    int nb_kp_ref;
    int nb_kp_cur;
    int nb_good; 
};

class Tracker {
public:
    Tracker(float sig = 1.6f,
            int edge = 10,
            float contrast = 0.02f,
            int nfeat = 1000, 
            int nLay = 3,
            int patch_size = 15);

    void computeReference(cv::Mat& img,
                          std::vector<cv::KeyPoint>& kp,
                          cv::Mat& des,
                          cv::Mat& gray,
                          std::vector<uint8_t>& Rchan);

    Displacement computeDisplacement(const std::vector<cv::KeyPoint>& kp1,
                                     const cv::Mat& des1,
                                     const cv::Mat& img1,
                                     const std::vector<uint8_t>& Rchan1,
                                     cv::Mat& img2,
                                     std::vector<cv::KeyPoint>& kp2,
                                     cv::Mat& des2,
                                     cv::Mat& gray2,
                                     std::vector<uint8_t>& Rchan2,
                                     std::vector<float>& rapports);

private:
    float sig_;
    int   edge_;
    float contrast_;
    int   nfeat_;
    int   nLay_;
    int   patch_size_;

    cv::Ptr<cv::SIFT> sift_;
    cv::BFMatcher     bf_;

    static float median(std::vector<float>& v);

    void kp_image(cv::Mat& img,
                  std::vector<cv::KeyPoint>& kp,
                  cv::Mat& des,
                  cv::Mat& gray,
                  std::vector<uint8_t>& Rchan);

    void matches(const cv::Mat& img1, const cv::Mat& des1,
                 const std::vector<cv::KeyPoint>& kp1, const std::vector<uint8_t>& Rchan1,
                 const cv::Mat& img2, const cv::Mat& des2,
                 const std::vector<cv::KeyPoint>& kp2, const std::vector<uint8_t>& Rchan2,
                 std::vector<std::vector<cv::DMatch>>& good,
                 double test_unicite,
                 std::vector<float>& rapports);
};
