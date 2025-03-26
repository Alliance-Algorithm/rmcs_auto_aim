#include <memory>

#include "core/transform_optimizer/armor/quadrilateral/quadrilateral.hpp"

#include "util/profile/profile.hpp"

using namespace rmcs_auto_aim::transform_optimizer;

Quadrilateral::Quadrilateral(ArmorPlate armorRef)
    : armor(std::move(armorRef)) {}

static constexpr double ratio(const auto& point) { return atan2(point.y, point.x); }

double Quadrilateral::operator-(Quadrilateral s2d) const {
    auto p1 = s2d.armor.points[0] - s2d.armor.points[1];
    auto p2 = s2d.armor.points[2] - s2d.armor.points[3];

    auto p3 = armor.points[1] - armor.points[0];
    auto p4 = armor.points[3] - armor.points[2];

    auto ratio1 = ratio(p1) - ratio(p3);
    auto ratio2 = ratio(p2) - ratio(p4);

    while (ratio1 >= std::numbers::pi)
        ratio1 -= std::numbers::pi;
    while (ratio1 <= -std::numbers::pi)
        ratio1 += std::numbers::pi;

    while (ratio2 >= std::numbers::pi)
        ratio2 -= std::numbers::pi;
    while (ratio2 <= -std::numbers::pi)
        ratio2 += std::numbers::pi;

    return (abs(ratio2) + abs(ratio1));
}

inline void Quadrilateral::draw(cv::InputOutputArray image, const cv::Scalar& color) const {
    cv::line(image, armor.points[0], armor.points[1], color);
    cv::line(image, armor.points[1], armor.points[2], color);
    cv::line(image, armor.points[2], armor.points[3], color);
    cv::line(image, armor.points[3], armor.points[0], color);
    cv::line(image, armor.points[2], armor.points[0], color);
}

struct Quadrilateral3d::Impl {

    explicit Impl(ArmorPlate3d const& armor3dRef)
        : armor3d(armor3dRef) {}

    // bull shit
    inline constexpr static const double NormalArmorWidth = 0.138, NormalArmorHeight = 0.056,
                                         LargerArmorWidth = 0.232, LargerArmorHeight = 0.056;

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

        std::vector<cv::Point3f> points{};
        auto& objectPoints = isLargeArmor ? LargeArmorObjectPoints : NormalArmorObjectPoints;

        for (int i = 0; i < 4; i++) {
            auto rotationInCamera = fast_tf::cast<rmcs_description::CameraLink>(
                rmcs_description::OdomImu::DirectionVector(
                    armor3d.rotation->toRotationMatrix() * objectPoints[i]),
                tf);
            auto pos = (*rotationInCamera + *positionInCamera) * 1000;
            points.emplace_back(-pos.y(), -pos.z(), pos.x());
        }
        return points;
    };

    Quadrilateral ToSquad(const rmcs_description::Tf& tf, bool isLargeArmor) const {
        auto objectPoints = get_objective_point(tf, isLargeArmor);

        auto intrinsic_parameters  = util::Profile::get_intrinsic_parameters();
        auto distortion_parameters = util::Profile::get_distortion_parameters();

        std::vector<cv::Point2f> imagePoints{};
        cv::Mat t = cv::Mat::zeros(3, 1, CV_32F), r = cv::Mat::zeros(3, 1, CV_32F);
        cv::projectPoints(
            objectPoints, t, r, intrinsic_parameters, distortion_parameters, imagePoints);

        return Quadrilateral(ArmorPlate(std::move(imagePoints), armor3d.id, isLargeArmor));
    }

    const ArmorPlate3d& armor3d;
};

Quadrilateral3d::Quadrilateral3d(ArmorPlate3d const& armor3dRef) {
    pimpl_ = std::make_unique<Impl>(armor3dRef);
}

Quadrilateral
    Quadrilateral3d::ToQuadrilateral(const rmcs_description::Tf& tf, bool isLargeArmor) const {
    return pimpl_->ToSquad(tf, isLargeArmor);
}

Quadrilateral3d::~Quadrilateral3d() = default;