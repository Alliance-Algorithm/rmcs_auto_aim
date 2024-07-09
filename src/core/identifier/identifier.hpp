/**
 * @file identifier.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-06-02
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */
#pragma once
#include <optional>

#include <opencv2/core/mat.hpp>

#include <rmcs_msgs/robot_color.hpp>

#include "core/identifier/armor/armor.hpp"
#include "core/identifier/buff/buff.hpp"

template <typename ResultType>
class Identifier {
public:
    virtual ~Identifier(){};
    template <class... Args>
    ResultType Identify(Args... args);
};

typedef Identifier<std::vector<auto_aim::ArmorPlate>> ArmorIdentifierInterface;
typedef Identifier<std::optional<auto_aim::BuffPlate>> BuffIdentifierInterface;
