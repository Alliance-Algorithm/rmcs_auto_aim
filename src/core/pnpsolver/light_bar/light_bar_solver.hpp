/**
 * @file armor_pnp_solver.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-06-02
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */
#pragma once
#include <opencv2/core/mat.hpp>
#include <vector>

#include "core/identifier/armor/armor.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"

namespace rmcs_auto_aim {

class LightBarSolver {
public:
    static std::vector<ArmorPlate3d>
        SolveAll(const std::vector<ArmorPlate>& armors, const rmcs_description::Tf& tf);

private:
    class StaticImpl;
};
} // namespace rmcs_auto_aim