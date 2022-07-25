#pragma once
#ifndef _TRIALSLAM_G2OLIB_H_
#define _TRIALSLAM_G2OLIB_H_

#include "trialSlam/common.h"
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel_impl.h>

TRIAL_SLAM_NAMESPACE_BEGIN

class poseVertex: public g2o::BaseVertex<6, SE3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        virtual void setToOriginImpl() override {
            _estimate = SE3();
        }

        virtual void oplusImpl(const double *update) override {
            Vec6 _update;
            _update << update[0], update[1], update[2], update[3], update[4], update[5];
            _estimate = SE3::exp(_update) * _estimate;
        }

        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }
};

class landmarkVertex: public g2o::BaseVertex<3, Vec3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        virtual void setToOriginImpl() override {
            _estimate = Vec3::Zero();
        }

        virtual void oplusImpl(const double *update) override {
            _estimate[0] += update[0];
            _estimate[1] += update[1];
            _estimate[2] += update[2];
        }

        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }
};

class projectionPoseEdge: public g2o::BaseUnaryEdge<2, Vec2, poseVertex> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        projectionPoseEdge(const Vec3& position, const Mat3x3& K)
            : _position(position), _K(K) {}

        virtual void computeError() override {
            const poseVertex* _v = static_cast<poseVertex*>(_vertices[0]);
            SE3 _T = _v -> estimate();
            Vec3 _p_uv = _K * (_T * _position);
            _p_uv /= _p_uv[2];
            _error = _measurement - _p_uv.head<2>();
        }

        virtual void linearizeOplus() override {
            const poseVertex* _v = static_cast<poseVertex*>(_vertices[0]);
            SE3 _T = _v -> estimate();
            Vec3 _p_c = _T * _position;
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double X = _p_c[0];
            double Y = _p_c[1];
            double Z = _p_c[2];
            double Zinv = 1.0 / (Z + 1e-10);
            double Zinv2 = Zinv * Zinv;

            _jacobianOplusXi <<
                -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2, -fx - fx * X * X * Zinv2, fx * Y * Zinv,
                0, -fy * Zinv, fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv;
        }

        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }

    private:
        Vec3 _position;
        Mat3x3 _K;
};

class projectionPoseAndLandmarkEdge: public g2o::BaseBinaryEdge<2, Vec2, poseVertex, landmarkVertex> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        projectionPoseAndLandmarkEdge(const Mat3x3& K, const SE3& pose)
            : _K(K), _pose(pose) {}

        virtual void computeError() override {
            const poseVertex* _v0 = static_cast<poseVertex*>(_vertices[0]);
            const landmarkVertex* _v1 = static_cast<landmarkVertex*>(_vertices[1]);
            SE3 _T = _v0 -> estimate();
            Vec3 _p_uv = _K * (_T * (_v1 -> estimate()));
            _p_uv /= _p_uv[2];
            _error = _measurement - _p_uv.head<2>();
        }

        virtual void linearizeOplus() override {
            const poseVertex* _v0 = static_cast<poseVertex*>(_vertices[0]);
            const landmarkVertex* _v1 = static_cast<landmarkVertex*>(_vertices[1]);
            SE3 _T = _v0 -> estimate();
            Vec3 _p_c = _pose * _T * (_v1 -> estimate());
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double X = _p_c[0];
            double Y = _p_c[1];
            double Z = _p_c[2];
            double Zinv = 1.0 / (Z + 1e-10);
            double Zinv2 = Zinv * Zinv;

            _jacobianOplusXi <<
                -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2, -fx - fx * X * X * Zinv2, fx * Y * Zinv,
                0, -fy * Zinv, fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv;

            _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * _pose.rotationMatrix() * _T.rotationMatrix();
        }

        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }

    private:
        Mat3x3 _K;
        SE3 _pose;
};

TRIAL_SLAM_NAMESPACE_END

#endif
