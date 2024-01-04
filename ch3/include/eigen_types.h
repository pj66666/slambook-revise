#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "sophus/se2.hpp"
#include "sophus/se3.hpp"



using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;

using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec6f = Eigen::Matrix<float, 6, 1>;

using Mat6d = Eigen::Matrix<double,6,6>;

using Quatd = Eigen::Quaterniond;
using Quatf = Eigen::Quaternionf;




// pose represented as sophus structs
using SE2 = Sophus::SE2d;
using SE2f = Sophus::SE2f;
using SE3 = Sophus::SE3d;
using SE3f = Sophus::SE3f;


using SO2 = Sophus::SO2d;
using SO3 = Sophus::SO3d;
