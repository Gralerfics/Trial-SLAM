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
#include <list>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

TRIAL_SLAM_NAMESPACE_BEGIN

#include "constants.h"
#include "typedefs.h"

TRIAL_SLAM_NAMESPACE_END

#endif
