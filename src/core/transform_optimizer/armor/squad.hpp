#pragma once

#include <cmath>
#include <fast_tf/impl/cast.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rmcs_description/tf_description.hpp>
#include <vector>

#include "core/identifier/armor/armor.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"
namespace rmcs_auto_aim::transform_optimizer {

struct Squad {

    explicit Squad(ArmorPlate armorRef)
        : armor(std::move(armorRef)) {}

    double operator-(Squad s2) const {
        double tmp = 0;
        for (int i = 0; i < 4; i++) {
            auto p = (s2.armor.points[i] - armor.points[i]);
            tmp += log(sqrt(p.x * p.x + p.y * p.y));
        }
        return tmp;
    }
    constexpr inline bool is_large_armor() const { return armor.is_large_armor; }
    inline static void darw_squad(
        cv::InputOutputArray image, const Squad& squad, const cv::Scalar& color, int thickness = 1,
        int lineType = cv::LINE_8, int shift = 0) {
        // cv::line(
        //     image, squad.armor.points[0], squad.armor.points[1], {255, 0, 0}, thickness,
        //     lineType, shift);
        // cv::line(
        //     image, squad.armor.points[1], squad.armor.points[2], {255, 255, 0}, thickness,
        //     lineType, shift);
        // cv::line(
        //     image, squad.armor.points[2], squad.armor.points[3], {0, 255, 0}, thickness,
        //     lineType, shift);
        // cv::line(
        //     image, squad.armor.points[3], squad.armor.points[0], {0, 0, 255}, thickness,
        //     lineType, shift);
        cv::line(
            image, squad.armor.points[0], squad.armor.points[1], color, thickness, lineType, shift);
        cv::line(
            image, squad.armor.points[1], squad.armor.points[2], color, thickness, lineType, shift);
        cv::line(
            image, squad.armor.points[2], squad.armor.points[3], color, thickness, lineType, shift);
        cv::line(
            image, squad.armor.points[3], squad.armor.points[0], color, thickness, lineType, shift);
    }

private:
    ArmorPlate armor;
};
struct Squad3d {

    const ArmorPlate3d& armor3d;

    explicit Squad3d(ArmorPlate3d const& armor3dRef)
        : armor3d(armor3dRef) {}

    inline static const Eigen::Vector3d AxisX{1, 0, 0};
    inline static const Eigen::Vector3d AxisY{0, 1, 0};

    // bull shit
    inline constexpr static const double NormalArmorWidth = 0.134, NormalArmorHeight = 0.056,
                                         LargerArmorWidth = 0.230, LargerArmorHeight = 0.056;

    inline const static std::vector<Eigen::Vector3d> LargeArmorObjectPoints = {
        Eigen::Vector3d(0.0, 0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight),
        Eigen::Vector3d(0.0, 0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight),
        Eigen::Vector3d(0.0, -0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight),
        Eigen::Vector3d(0.0, -0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight)};
    inline const static std::vector<Eigen::Vector3d> NormalArmorObjectPoints = {
        Eigen::Vector3d(0.0, 0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight),
        Eigen::Vector3d(0.0, 0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight),
        Eigen::Vector3d(0.0, -0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight),
        Eigen::Vector3d(0.0, -0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight)};

    inline std::vector<cv::Point3f>
        get_objective_point(rmcs_description::Tf const& tf, bool isLargeArmor) const {
        auto positionInCamera = fast_tf::cast<rmcs_description::CameraLink>(armor3d.position, tf);
        auto rotationInCamera = fast_tf::cast<rmcs_description::CameraLink>(armor3d.rotation, tf);

        std::vector<cv::Point3f> points{};
        auto& objectPoints = isLargeArmor ? LargeArmorObjectPoints : NormalArmorObjectPoints;

        for (int i = 0; i < 4; i++) {
            auto pos =
                ((rotationInCamera->toRotationMatrix() * objectPoints[i]) + *positionInCamera)
                * 1000;
            points.emplace_back(-pos.y(), -pos.z(), pos.x());
        }
        return points;
    };
    Squad ToSquad(
        const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, const rmcs_description::Tf& tf,
        bool isLargeArmor) const {
        auto objectPoints = get_objective_point(tf, isLargeArmor);

        std::vector<cv::Point2f> imagePoints{};
        cv::Mat t = cv::Mat::zeros(3, 1, CV_32F), r = cv::Mat::zeros(3, 1, CV_32F);
        cv::projectPoints(objectPoints, t, r, cameraMatrix, distCoeffs, imagePoints);
        return Squad(ArmorPlate(std::move(imagePoints), armor3d.id, isLargeArmor));
    }
    inline static void darw_squad(
        const double& fx, const double& fy, const double& cx, const double& cy, const double& k1,
        const double& k2, const double& k3, const rmcs_description::Tf& tf,
        cv::InputOutputArray image, const Squad3d& squad3d, bool is_large_armor,
        const cv::Scalar& color, int thickness = 1, int lineType = cv::LINE_8, int shift = 0) {

        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        cv::Mat distCoeffs   = (cv::Mat_<double>(1, 5) << k1, k2, 0, 0, k3);

        auto objectPoints = squad3d.get_objective_point(tf, is_large_armor);

        int i = 0;
        for (auto& armor : objectPoints) {
            std::string text = "(" + std::to_string(armor.x) + ", " + std::to_string(armor.y) + ", "
                             + std::to_string(armor.z) + ")";
            cv::putText(image, text, {100 + (++i) * 100, 100 + (++i) * 50}, 1, 1, {0, 0, 255});
        }

        Squad::darw_squad(
            image, squad3d.ToSquad(cameraMatrix, distCoeffs, tf, is_large_armor), color, thickness,
            lineType, shift);
    }
};
} // namespace rmcs_auto_aim::transform_optimizer