#pragma once
#ifndef TRIALSLAM_COMMON_H_
#define TRIALSLAM_COMMON_H_

#define TRIAL_SLAM_NAMESPACE_BEGIN namespace trialSlam {
#define TRIAL_SLAM_NAMESPACE_END }

#include <iostream>
#include <chrono>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <list>
#include <vector>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXxX;
typedef Eigen::Matrix<double, 2, 2> Mat2x2;
typedef Eigen::Matrix<double, 3, 3> Mat3x3;
typedef Eigen::Matrix<double, 4, 4> Mat4x4;

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 6, 1> Vec6;

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <glog/logging.h>

TRIAL_SLAM_NAMESPACE_BEGIN

#include "constants.h"

TRIAL_SLAM_NAMESPACE_END

#endif
