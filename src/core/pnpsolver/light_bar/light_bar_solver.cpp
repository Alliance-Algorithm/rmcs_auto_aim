
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <fast_tf/impl/cast.hpp>
#include <rmcs_description/tf_description.hpp>
#include <tuple>

#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/pnpsolver/light_bar/light_bar_solver.hpp"
#include "light_bar_solver.hpp"
#include "line.hpp"
#include "util/image_viewer/image_viewer.hpp"
#include "util/math.hpp"
#include "util/optimizer/fibonacci.hpp"
#include "util/profile/profile.hpp"
#include "util/utils.hpp"

using namespace rmcs_auto_aim;

class LightBarSolver::StaticImpl {
public:
    static std::vector<ArmorPlate3d>
        SolveAll(const std::vector<ArmorPlate>& armors, const rmcs_description::Tf& tf) {
        auto camera_rotation_in_odom = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::Rotation(Eigen::Quaterniond::Identity()), tf);
        auto camera_yaw_in_odom = util::math::get_yaw_from_quaternion(*camera_rotation_in_odom);

        const Eigen::AngleAxis rotation = Eigen::AngleAxis(
            15. / 180.0 * std::numbers::pi,
            *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitY()));
        std::vector<ArmorPlate3d> result;
        for (auto& armor : armors) {
            const auto& [top, button, dir, top1, button1] = get_full_light_bar(armor);
            const Line image_line{top, button};
            const Line image_line1{top1, button1};
            Eigen::Vector3d camera_vec;
            Eigen::Vector3d camera_vec1;
            util::compute_yaw_pitch_from_point(
                (image_line.button_ + image_line.top_) / 2,
                util::Profile::get_intrinsic_parameters(),
                util::Profile::get_distortion_parameters(), camera_vec, 3);
            util::compute_yaw_pitch_from_point(
                (image_line1.button_ + image_line1.top_) / 2,
                util::Profile::get_intrinsic_parameters(),
                util::Profile::get_distortion_parameters(), camera_vec1, 3);
            const auto target_yaw = util::optimizer::fibonacci(
                camera_yaw_in_odom - std::numbers::pi / 2,
                camera_yaw_in_odom + std::numbers::pi / 2, 1e-4,
                [&image_line, &image_line1, &rotation, &armor, &tf, &camera_vec,
                 &camera_vec1](const double& yaw) {
                    auto line_rotation = set_armor3d_angle(rotation, yaw);
                    Line3d line3d{
                        *line_rotation, camera_vec,
                        (armor.is_large_armor ? LargerArmorHeight : NormalArmorHeight) / 1000.0};
                    Line3d line3d1{
                        *line_rotation, camera_vec1,
                        (armor.is_large_armor ? LargerArmorHeight : NormalArmorHeight) / 1000.0};
                    return line3d.to_line_2d(tf).angle_distance(image_line);
                    //  + line3d1.to_line_2d(tf).angle_distance(image_line1) * 0.5;
                });
            const auto target_distance = util::optimizer::fibonacci(
                0, 20, 1e-3,
                [&image_line, &rotation, &armor, &tf, target_yaw,
                 &camera_vec](const double& distance) {
                    auto line_rotation = set_armor3d_angle(rotation, target_yaw);
                    auto tmp           = camera_vec * distance;
                    Line3d line3d{
                        *line_rotation, tmp,
                        (armor.is_large_armor ? LargerArmorHeight : NormalArmorHeight) / 1000.0};
                    return line3d.to_line_2d(tf).length_distance(image_line);
                });
            auto armor_angle = set_armor3d_angle(rotation, target_yaw);
            Line3d line3d{
                *armor_angle, camera_vec * target_distance,
                (armor.is_large_armor ? LargerArmorHeight : NormalArmorHeight) / 1000.0};
            util::ImageViewer::draw(line3d.to_line_2d(tf), {255, 255, 0});
            result.emplace_back(
                armor.id,
                rmcs_description::OdomImu ::Position(
                    *fast_tf::cast<rmcs_description::OdomImu>(
                        rmcs_description::CameraLink::Position{camera_vec * target_distance}, tf)
                    + *armor_angle * dir
                          * (armor.is_large_armor ? LargerArmorWidth : NormalArmorWidth) / 1000.0
                          / 2),
                armor_angle);
        }
        return result;
    }

private:
    static inline rmcs_description::OdomImu::Rotation
        set_armor3d_angle(const Eigen::AngleAxis<double>& inOutArmor3d, const double& angle) {
        return rmcs_description::OdomImu::Rotation(
            Eigen::AngleAxis(angle, Eigen::Vector3d::UnitZ()) * inOutArmor3d);
    }

    inline constexpr static const double MaxArmorDistance = 15.0;

    inline constexpr static const double NormalArmorWidth = 134, NormalArmorHeight = 56,
                                         LargerArmorWidth = 230, LargerArmorHeight = 56;
    inline static const std::vector<cv::Point3d> LargeArmorObjectPoints = {
        cv::Point3d(-0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight, 0.0f),
        cv::Point3d(-0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight, 0.0f),
        cv::Point3d(0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight, 0.0f),
        cv::Point3d(0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight, 0.0f)};
    inline static const std::vector<cv::Point3d> NormalArmorObjectPoints = {
        cv::Point3d(-0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0.0f),
        cv::Point3d(-0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight, 0.0f),
        cv::Point3d(0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight, 0.0f),
        cv::Point3d(0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0.0f)};
    inline static std::tuple<
        const cv::Point2f, const cv::Point2f, const Eigen::Vector3d, const cv::Point2f,
        const cv::Point2f>
        get_full_light_bar(const ArmorPlate& armor) {
        cv::Point2f l1 = armor.points[0] - armor.points[1];
        return abs(util::math::clamp_pm_pi(util::math::ratio(l1))) > std::numbers::pi / 2
                 ? std::tuple<
                       cv::Point2f, cv::Point2f,
                       const Eigen::
                           Vector3d, const cv::Point2f,
        const cv::Point2f>{armor.points[0], armor.points[1], -Eigen::Vector3d::UnitY(),armor.points[3], armor.points[2]}
                 : std::tuple<cv::Point2f, cv::Point2f, const Eigen::Vector3d, const cv::Point2f,
        const cv::Point2f>{
                       armor.points[3], armor.points[2], Eigen::Vector3d::UnitY(),armor.points[0], armor.points[1]};
    }
};

std::vector<ArmorPlate3d> LightBarSolver::SolveAll(
    const std::vector<ArmorPlate>& armors, const rmcs_description::Tf& tf) {
    return LightBarSolver::StaticImpl::SolveAll(armors, tf);
}
