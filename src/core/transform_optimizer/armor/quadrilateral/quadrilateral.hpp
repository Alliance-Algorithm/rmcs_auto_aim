#pragma once

#include <cmath>
#include <memory>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include <fast_tf/impl/cast.hpp>
#include <rmcs_description/tf_description.hpp>

#include "core/identifier/armor/armor.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"

#include "util/image_viewer/image_viewer.hpp"

namespace rmcs_auto_aim::transform_optimizer {

struct Quadrilateral final : util::IAutoAimDrawable {

    explicit Quadrilateral(ArmorPlate armorRef);

    double operator-(Quadrilateral s2d) const;

    void draw(cv::InputOutputArray image, const cv::Scalar& color) const final;

    constexpr inline bool is_large_armor() const { return armor.is_large_armor; }

private:
    ArmorPlate armor;
};
struct Quadrilateral3d {

    explicit Quadrilateral3d(ArmorPlate3d const& armor3dRef);
    ~Quadrilateral3d();

    Quadrilateral ToQuadrilateral(const rmcs_description::Tf& tf, bool isLargeArmor) const;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
};
} // namespace rmcs_auto_aim::transform_optimizer