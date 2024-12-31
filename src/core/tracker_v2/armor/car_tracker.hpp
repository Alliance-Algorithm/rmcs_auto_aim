
#include <memory>
#include <numbers>
#include <rmcs_description/tf_description.hpp>
#include <robot_id.hpp>
#include <tuple>
#include <vector>

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>

#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/tracker_v2/armor/car_kf.hpp"

namespace rmcs_auto_aim::tracker2 {
class CarTracker {
public:
    CarTracker()
        : car_kf_() {}

    void update_car(const Eigen::MatrixXd& X_k, const double& dt) {
        car_kf_.Update(X_k, Eigen::VectorXd::Zero(8), dt);
    }
    std::shared_ptr<std::vector<ArmorPlate3d>> get_armor(double dt = 0) {
        auto armors = std::make_shared<std::vector<ArmorPlate3d>>();
        auto X      = car_kf_.OutPut();

        Eigen::Vector3d center{X(0) + X(1) * dt, X(2) + X(3) * dt, X(4) * dt};

        auto forward_car   = Eigen::AngleAxisd(X(5), Eigen::Vector3d::UnitZ());
        auto forward_armor = Eigen::AngleAxisd(X(5) + std::numbers::pi, Eigen::Vector3d::UnitZ());
        armors->emplace_back(
            rmcs_msgs::ArmorID::Unknown,
            rmcs_description::OdomImu::DirectionVector(
                center + forward_car.toRotationMatrix() * Eigen::Vector3d::UnitX() * l1
                + Eigen::Vector3d::UnitZ() * z1),
            rmcs_description::OdomImu::Rotation(forward_armor));

        forward_car = Eigen::AngleAxisd(X(5) + std::numbers::pi / 2, Eigen::Vector3d::UnitZ());
        forward_armor =
            Eigen::AngleAxisd(X(5) + 3 * std::numbers::pi / 2, Eigen::Vector3d::UnitZ());
        armors->emplace_back(
            rmcs_msgs::ArmorID::Unknown,
            rmcs_description::OdomImu::DirectionVector(
                center + forward_car.toRotationMatrix() * Eigen::Vector3d::UnitX() * l2
                + Eigen::Vector3d::UnitZ() * z2),
            rmcs_description::OdomImu::Rotation(forward_armor));

        forward_car   = Eigen::AngleAxisd(X(5) + std::numbers::pi, Eigen::Vector3d::UnitZ());
        forward_armor = Eigen::AngleAxisd(X(5) + 2 * std::numbers::pi, Eigen::Vector3d::UnitZ());
        armors->emplace_back(
            rmcs_msgs::ArmorID::Unknown,
            rmcs_description::OdomImu::DirectionVector(
                center + forward_car.toRotationMatrix() * Eigen::Vector3d::UnitX() * l1
                + Eigen::Vector3d::UnitZ() * z3),
            rmcs_description::OdomImu::Rotation(forward_armor));

        forward_car = Eigen::AngleAxisd(X(5) + 3 * std::numbers::pi / 2, Eigen::Vector3d::UnitZ());
        forward_armor =
            Eigen::AngleAxisd(X(5) + 5 * std::numbers::pi / 2, Eigen::Vector3d::UnitZ());
        armors->emplace_back(
            rmcs_msgs::ArmorID::Unknown,
            rmcs_description::OdomImu::DirectionVector(
                center + forward_car.toRotationMatrix() * Eigen::Vector3d::UnitX() * l2
                + Eigen::Vector3d::UnitZ() * z4),
            rmcs_description::OdomImu::Rotation(forward_armor));

        return armors;
    }

    void update_frame(double l1, double l2) {
        this->l1 = l1;
        this->l2 = l2;
    };

    void update_z(const double& z1, const double& z2, const double& z3, const double& z4) {
        this->z1 = z1;
        this->z2 = z2;
        this->z3 = z3;
        this->z4 = z4;
    };
    std::tuple<double, double> get_frame() { return {l1, l2}; }

private:
    CarKF car_kf_;
    double l1 = 0.6, l2 = 0.6;
    double z1 = 0, z2 = 0, z3 = 0, z4 = 0;
};
} // namespace rmcs_auto_aim::tracker2