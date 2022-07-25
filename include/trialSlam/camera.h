#pragma once
#ifndef _TRIALSLAM_CAMERA_H_
#define _TRIALSLAM_CAMERA_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Camera {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Camera> Ptr;

        Camera() {}

        Camera(double fx, double fy, double cx, double cy,
               double k1, double k2, double k3, double p1, double p2,
               int camera_index = 0, int width = 640, int height = 480)
            : _fx(fx), _fy(fy), _cx(cx), _cy(cy),
            _k1(k1), _k2(k2), _k3(k3), _p1(p1), _p2(p2),
            _camera_index(camera_index), _width(width), _height(height) {}

        Vec3 world2camera(const Vec3& p_w, const SE3& T_cw) const;

        Vec3 camera2world(const Vec3& p_c, const SE3& T_cw) const;

        Vec2 camera2pixel(const Vec3& p_c) const;

        Vec3 pixel2camera(const Vec2& p_uv, double depth = 1) const;

        Vec2 world2pixel(const Vec3& p_w, const SE3& T_cw) const;

        Vec3 pixel2world(const Vec2& p_uv, const SE3& T_cw, double depth = 1) const;

        bool open();

        void close();

        bool capture(cv::Mat& captureMat);

        void setCameraIndex(int camera_index) { _camera_index = camera_index; }

        Mat3x3 getK_Eigen() const {
            Mat3x3 _K;
            _K << _fx, 0, _cx, 0, _fy, _cy, 0, 0, 1;
            return _K;
        }

        cv::Mat getK_CV() const {
            cv::Mat _K;
            cv::eigen2cv(getK_Eigen(), _K);
            return _K;
        }

        std::string _toString() const;

        friend std::ostream& operator <<(std::ostream& output, const Camera& camera);

    public:
        double _fx = 0, _fy = 0, _cx = 0, _cy = 0;
        double _k1 = 0, _k2 = 0, _k3 = 0, _p1 = 0, _p2 = 0;
        int _width = 640, _height = 480;
        int _camera_index = 0;

        std::shared_ptr<cv::VideoCapture> _capture = nullptr;
};

TRIAL_SLAM_NAMESPACE_END

#endif
