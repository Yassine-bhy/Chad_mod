#pragma once

#include <vector>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>



// Contient le déplacement global + infos sur les matches
struct Displacement {
    float X;
    float Y;
    float Z;
    int nb_kp_ref;
    int nb_kp_cur;
    int nb_good; 
};


// Classe principale gérant SIFT + matching + déplacement
class Tracker {
public:
    Tracker(float sig = 1.6f,
            int edge = 12,
            float contrast = 0.02f,
            int nfeat = 1000, 
            int nLay = 3,
            int patch_size = 15);

    // Prépare l’image de référence (kp, des, gris, rouge)
    void computeReference(cv::Mat& img,
                          std::vector<cv::KeyPoint>& kp,
                          cv::Mat& des,
                          cv::Mat& gray,
                          std::vector<float>& Rchan);


    // Calcule déplacement entre référence et image courante
    Displacement computeDisplacement(const std::vector<cv::KeyPoint>& kp1,
                                     const cv::Mat& des1,
                                     const cv::Mat& img1,
                                     const std::vector<float>& Rchan1,
                                     cv::Mat& img2,
                                     std::vector<cv::KeyPoint>& kp2,
                                     cv::Mat& des2,
                                     cv::Mat& gray2,
                                     std::vector<float>& Rchan2,
                                     std::vector<float>& dx_values);

private:
    float sig_;
    int   edge_;
    float contrast_;
    int   nfeat_;
    int   nLay_;
    int   patch_size_;

    cv::Ptr<cv::SIFT> sift_;
    cv::BFMatcher     bf_;


    // Renvoie la médiane d’un vecteur
    static float median(std::vector<float>& v);


    // Convertit en gris + canal rouge + calcule SIFT
    void kp_image(cv::Mat& img,
                  std::vector<cv::KeyPoint>& kp,
                  cv::Mat& des,
                  cv::Mat& gray,
                  std::vector<float>& Rchan);


    // Match SIFT + ratio test
    void matches(const cv::Mat& img1, const cv::Mat& des1,
                 const std::vector<cv::KeyPoint>& kp1, const std::vector<float>& Rchan1,
                 const cv::Mat& img2, const cv::Mat& des2,
                 const std::vector<cv::KeyPoint>& kp2, const std::vector<float>& Rchan2,
                 std::vector<std::vector<cv::DMatch>>& good,
                 double test_unicite);
};
