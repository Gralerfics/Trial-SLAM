#pragma once
#ifndef TRIALSLAM_TYPEDEFS_H_
#define TRIALSLAM_TYPEDEFS_H_

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXxX;
typedef Eigen::Matrix<double, 3, 3> Mat3x3;
typedef Eigen::Matrix<double, 4, 4> Mat4x4;

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 2, 1> Vec2;

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

// #define Vec2Point(v) (cv::Point2f(v.x, v.y))
// #define Point2Vec(p) (Vec2(p.x, p.y))

#endif
