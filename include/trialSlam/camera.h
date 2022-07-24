#pragma once
#ifndef TRIALSLAM_PERIPHERAL_CAMERA_H
#define TRIALSLAM_PERIPHERAL_CAMERA_H

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class MonoCamera {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // Constructor
            MonoCamera() {}

            MonoCamera(double fx, double fy, double cx, double cy,
                    double k1, double k2, double k3, double p1, double p2,
                    int camera_index = 0)
                : _fx(fx), _fy(fy), _cx(cx), _cy(cy),
                _k1(k1), _k2(k2), _k3(k3), _p1(p1), _p2(p2),
                _camera_index(camera_index) {}

        // Converter
            Vec3 world2camera(const Vec3& p_w, const SE3& pose) const;

            Vec3 camera2world(const Vec3& p_c, const SE3& pose) const;

            Vec2 camera2pixel(const Vec3& p_c) const;

            Vec3 pixel2camera(const Vec2& p_uv, double depth = 1) const;

            Vec2 world2pixel(const Vec3& p_w, const SE3& pose) const;

            Vec3 pixel2world(const Vec2& p_uv, const SE3& pose, double depth = 1) const;

        // Hardware
            bool open();

            void close();

            bool capture(cv::Mat& captureMat);

            void setCameraIndex(int camera_index) { _camera_index = camera_index; }

        // Debug
            std::string _toString() const;

            friend std::ostream& operator <<(std::ostream& output, const MonoCamera& camera);
    
    private:
        double _fx = 0, _fy = 0, _cx = 0, _cy = 0;
        double _k1 = 0, _k2 = 0, _k3 = 0, _p1 = 0, _p2 = 0;
        int _camera_index = 0;

        std::shared_ptr<cv::VideoCapture> _capture = nullptr;
};

TRIAL_SLAM_NAMESPACE_END

#endif
