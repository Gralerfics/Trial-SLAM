#pragma once
#ifndef TRIALSLAM_TYPEDEFS_H_
#define TRIALSLAM_TYPEDEFS_H_

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;
typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 4, 4> Mat44;

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 2, 1> Vec2;

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

#endif
