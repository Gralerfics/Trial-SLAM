#pragma once
#ifndef _TRIALSLAM_VISUAL_ODOMETRY_H_
#define _TRIALSLAM_VISUAL_ODOMETRY_H_

#include "trialSlam/common.h"
#include "trialSlam/camera.h"
#include "trialSlam/frontend.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class VisualOdometry {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;

        VisualOdometry() {}

        void setFrontend(Frontend::Ptr frontend) { _frontend = frontend; }

        bool initialize();

        void execute();

        void stop();

    private:
        Frontend::Ptr _frontend = nullptr;
};

TRIAL_SLAM_NAMESPACE_END

#endif
