#include "chad_mod.h"

#include <algorithm>
#include <cmath>

Tracker::Tracker(float sig,
                 int edge,
                 float contrast,
                 int nfeat,
                 int nLay,
                 int patch_size)
    : sig_(sig),
      edge_(edge),
      contrast_(contrast),
      nfeat_(nfeat),
      nLay_(nLay),
      patch_size_(patch_size),
      bf_(cv::NORM_L2, false)
{
    sift_ = cv::SIFT::create(
        nfeat_,
        nLay_,
        contrast_,
        edge_,
        sig_
    );
}

float Tracker::median(std::vector<float>& v) {
    if (v.empty())
        return 0.0f;

    std::sort(v.begin(), v.end());
    size_t n = v.size();
    if (n % 2 == 1)
        return v[n/2];
    return 0.5f * (v[n/2 - 1] + v[n/2]);
}

void Tracker::kp_image(cv::Mat& img,
                       std::vector<cv::KeyPoint>& kp,
                       cv::Mat& des,
                       cv::Mat& gray,
                       std::vector<float>& Rchan)
{
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    Rchan.resize(img.rows * img.cols);

    for (int y = 0; y < img.rows; ++y) {
        const cv::Vec3b* row = img.ptr<cv::Vec3b>(y);
        for (int x = 0; x < img.cols; ++x) {
            float R = row[x][2];
            Rchan[y * img.cols + x] = R;
        }
    }

    sift_->detectAndCompute(gray, cv::noArray(), kp, des);
}

void Tracker::matches(const cv::Mat& img1, const cv::Mat& des1,
                      const std::vector<cv::KeyPoint>& kp1, const std::vector<float>& Rchan1,
                      const cv::Mat& img2, const cv::Mat& des2,
                      const std::vector<cv::KeyPoint>& kp2, const std::vector<float>& Rchan2,
                      std::vector<std::vector<cv::DMatch>>& good,
                      double test_unicite,
                      std::vector<float>& rapports)
{
    std::vector<std::vector<cv::DMatch>> knn_matches;
    bf_.knnMatch(des1, des2, knn_matches, 2);

    good.clear();
    rapports.clear();
    rapports.reserve(knn_matches.size());

    for (auto& pair : knn_matches) {
        if (pair.size() < 2) continue;
        const cv::DMatch& m = pair[0];
        const cv::DMatch& n = pair[1];
        if (m.distance < test_unicite * n.distance) {
            good.push_back({m});
        }
    }
}

void Tracker::computeReference(cv::Mat& img,
                               std::vector<cv::KeyPoint>& kp,
                               cv::Mat& des,
                               cv::Mat& gray,
                               std::vector<float>& Rchan)
{
    kp_image(img, kp, des, gray, Rchan);
}

Displacement Tracker::computeDisplacement(const std::vector<cv::KeyPoint>& kp1,
                                          const cv::Mat& des1,
                                          const cv::Mat& img1,
                                          const std::vector<float>& Rchan1,
                                          cv::Mat& img2,
                                          std::vector<cv::KeyPoint>& kp2,
                                          cv::Mat& des2,
                                          cv::Mat& gray2,
                                          std::vector<float>& Rchan2,
                                          std::vector<float>& rapports)
{
    std::vector<std::vector<cv::DMatch>> good;

    kp_image(img2, kp2, des2, gray2, Rchan2);

    matches(img1, des1, kp1, Rchan1,
            img2, des2, kp2, Rchan2,
            good, 0.75, rapports);

    Displacement disp{};
    disp.X = 0.0f;
    disp.Y = 0.0f;
    disp.Z = 0.0f;
    disp.nb_kp_ref = static_cast<int>(kp1.size());
    disp.nb_kp_cur = static_cast<int>(kp2.size());
    disp.nb_good   = static_cast<int>(good.size());


    if (good.size() < 5) {
        return disp;
    }

    std::vector<float> dy_values, dz_values;
    dy_values.reserve(good.size());
    dz_values.reserve(good.size());

    int half_patch = patch_size_ / 2;
    std::vector<float> patch_reds1;
    std::vector<float> patch_reds2;

    for (const auto& g : good) {
        const cv::DMatch& m = g[0];
        int idx1 = m.queryIdx;
        int idx2 = m.trainIdx;
        float x1 = kp1[idx1].pt.x;
        float y1 = kp1[idx1].pt.y;
        float x2 = kp2[idx2].pt.x;
        float y2 = kp2[idx2].pt.y;

        dy_values.push_back(static_cast<float>(x2 - x1));
        dz_values.push_back(static_cast<float>(y2 - y1));

        patch_reds1.clear();
        patch_reds2.clear();

        for (int dy = -half_patch; dy <= half_patch; ++dy) {
            int yy1 = static_cast<int>(y1) + dy;
            if (yy1 < 0 || yy1 >= img1.rows) continue;
            for (int dx = -half_patch; dx <= half_patch; ++dx) {
                int xx1 = static_cast<int>(x1) + dx;
                if (xx1 < 0 || xx1 >= img1.cols) continue;
                patch_reds1.push_back(
                    static_cast<float>(Rchan1[yy1 * img1.cols + xx1])
                );
            }

            int yy2 = static_cast<int>(y2) + dy;
            if (yy2 < 0 || yy2 >= img2.rows) continue;
            for (int dx = -half_patch; dx <= half_patch; ++dx) {
                int xx2 = static_cast<int>(x2) + dx;
                if (xx2 < 0 || xx2 >= img2.cols) continue;
                patch_reds2.push_back(
                    static_cast<float>(Rchan2[yy2 * img2.cols + xx2])
                );
            }
        }

        if (!patch_reds1.empty() && !patch_reds2.empty()) {
            float median_patch1 = median(patch_reds1);
            float median_patch2 = median(patch_reds2);

            if (median_patch2 > 0.0f) {
                rapports.push_back(median_patch1 - median_patch2);
            }
        }
    }

    if (dy_values.empty() || dz_values.empty() || rapports.empty()) {
        return disp;
    }

    disp.X = median(rapports);
    disp.Y = median(dy_values);
    disp.Z = median(dz_values);

    return disp;
}
