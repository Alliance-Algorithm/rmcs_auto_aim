#include <ceres/jet.h>
#include <cmath>
#include <opencv2/core.hpp>
#include <utility>
#include <vector>

#include "ceres/ceres.h"
#include "opencv2/opencv.hpp"
#include "util/math.hpp"

namespace re_projection {
using namespace std;
using namespace cv;
struct ReProjectionLineRatioError {
    ReProjectionLineRatioError(
        vector<Point3f> object_point, vector<Point2f> image_point, Mat camera_matrix,
        Mat dist_coeffs)
        : object_point_(std::move(object_point))
        , image_point_(std::move(image_point))
        , camera_matrix_(std::move(camera_matrix))
        , dist_coeffs_(std::move(dist_coeffs)) {}

    template <typename T>
    bool operator()(const T* const rvec, const T* const tvec, T* residuals) const {
        // 转换旋转向量和平移向量
        Mat rvec_mat(3, 1, CV_64F, const_cast<T*>(rvec));
        Mat tvec_mat(3, 1, CV_64F, const_cast<T*>(tvec));

        // 3D点转换为投影点
        vector<Point2f> projected_points;
        projectPoints(
            object_point_, rvec_mat, tvec_mat, camera_matrix_, dist_coeffs_, projected_points);
        double ratio1 = rmcs_auto_aim::util::math::ratio(image_point_[0] - projected_points[0])
                      - rmcs_auto_aim::util::math::ratio(image_point_[1] - projected_points[1]);
        while (ratio1 >= std::numbers::pi)
            ratio1 -= std::numbers::pi;
        while (ratio1 <= -std::numbers::pi)
            ratio1 += std::numbers::pi;
        // 计算重投影误差+
        residuals[0] = T();

        if (!ceres::IsFinite(residuals[0])) {
            std::cerr << "Error: Non-finite residuals detected. 1" << std::endl;
            return false;
        }

        return true;
    }

private:
    vector<Point3f> object_point_;
    vector<Point2f> image_point_;
    Mat camera_matrix_;
    Mat dist_coeffs_;
};
struct ReProjectionPointError {
    ReProjectionPointError(
        vector<Point3f> object_point, Point2f image_point, Mat camera_matrix, Mat dist_coeffs)
        : object_point_(std::move(object_point))
        , image_point_(std::move(image_point))
        , camera_matrix_(std::move(camera_matrix))
        , dist_coeffs_(std::move(dist_coeffs)) {}

    template <typename T>
    bool operator()(const T* const rvec, const T* const tvec, T* residuals) const {
        // 转换旋转向量和平移向量
        Mat rvec_mat(3, 1, CV_64F, const_cast<T*>(rvec));
        Mat tvec_mat(3, 1, CV_64F, const_cast<T*>(tvec));

        // 3D点转换为投影点
        vector<Point2f> projected_points;
        projectPoints(
            object_point_, rvec_mat, tvec_mat, camera_matrix_, dist_coeffs_, projected_points);

        // 计算重投影误差+
        residuals[0] = T(image_point_.x - projected_points[0].x);
        residuals[1] = T(image_point_.y - projected_points[0].y);

        if (!ceres::IsFinite(residuals[0]) || !ceres::IsFinite(residuals[1])) {
            std::cerr << "Error: Non-finite residuals detected. 2" << std::endl;
            return false;
        }

        return true;
    }

private:
    vector<Point3f> object_point_;
    Point2f image_point_;
    Mat camera_matrix_;
    Mat dist_coeffs_;
};
} // namespace re_projection