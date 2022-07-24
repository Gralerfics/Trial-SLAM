#pragma once
#ifndef _TRIALSLAM_VISUAL_ODOMETRY_H_
#define _TRIALSLAM_VISUAL_ODOMETRY_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class VisualOdometry {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;

        VisualOdometry() {}

    private:

};

TRIAL_SLAM_NAMESPACE_END

#endif
