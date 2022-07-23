#pragma once
#ifndef TRIALSLAM_FRAME_H_
#define TRIALSLAM_FRAME_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Frame() {}

    private:
        SE3 _cam_pose;
};

TRIAL_SLAM_NAMESPACE_END

#endif
