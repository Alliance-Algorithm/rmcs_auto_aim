#pragma once

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <numbers>
#include <vector>

#include "core/identifier/armor/armor.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/pnpsolver/armor/armor_pnp_solver.hpp"
#include "core/pnpsolver/light_bar/light_bar_solver.hpp"
#include "core/transform_optimizer/armor/armor.hpp"

class FusionSolver {
public:
    static std::vector<rmcs_auto_aim::ArmorPlate3d> SolveAll(
        const std::vector<rmcs_auto_aim::ArmorPlate>& armors, const rmcs_description::Tf& tf) {

        auto armor3d_with_light_bar = rmcs_auto_aim::LightBarSolver::SolveAll(armors, tf);
        auto armor3d_with_ippe      = rmcs_auto_aim::ArmorPnPSolver::SolveAll(armors, tf);

        if (armor3d_with_ippe.size() != armor3d_with_light_bar.size()) {
            return armor3d_with_ippe;
        }

        rmcs_auto_aim::transform_optimizer::armor_transform_optimize(armors, armor3d_with_ippe, tf);
        auto camera_x = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::DirectionVector(Eigen::Vector3d::UnitX()), tf);

        for (size_t i = 0; i < armor3d_with_light_bar.size(); i++) {
            // std::cerr << *armor3d_with_light_bar[i].rotation << "\n"
            //           << *armor3d_with_ippe[i].rotation << "\n---\n";
            double ratio = (*armor3d_with_light_bar[i].rotation * Eigen::Vector3d::UnitX())
                               .dot(camera_x->normalized());
            ratio                               = (std::clamp(ratio, 0.8, 0.9) - 0.8) * 10;
            *armor3d_with_light_bar[i].position = (1 - ratio) * *armor3d_with_light_bar[i].position
                                                + ratio * *armor3d_with_ippe[i].position;
            *armor3d_with_light_bar[i].rotation =
                armor3d_with_light_bar[i].rotation->slerp(ratio, *armor3d_with_ippe[i].rotation);
        }
        return armor3d_with_light_bar;
    }
};