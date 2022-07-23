#pragma once
#ifndef TRIALSLAM_VO_H_
#define TRIALSLAM_VO_H_

#include "trialSlam/common.h"
#include "trialSlam/camera.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class VisualOdometry {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VisualOdometry() {}
        
        std::shared_ptr<Camera> getCameraPtr() const;
        Camera& getCamera() const;
        int getCameraType() const;
        void setCamera(std::shared_ptr<Camera> camera, int camera_type);

        bool initialize();

        void execute();

    private:
        int _camera_type = VO_CAMERA_TYPE_UNDEFINED;
        std::shared_ptr<Camera> _camera = nullptr;
};

TRIAL_SLAM_NAMESPACE_END

#endif
