#pragma once

#include <memory>           // 智能指针，用于管理动态分配的内存
#include "eigen_types.h"

namespace pj {

// IMU 读数
struct IMU {
    IMU() = default;
    IMU(double t, const Vec3d& gyro, const Vec3d& acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

    double timestamp_ = 0.0;
    Vec3d gyro_ = Vec3d::Zero();
    Vec3d acce_ = Vec3d::Zero();
};
}  // namespace pj

using IMUPtr = std::shared_ptr<pj::IMU>;

