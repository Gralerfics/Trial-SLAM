#pragma once
#ifndef _TRIALSLAM_LANDMARK_H_
#define _TRIALSLAM_LANDMARK_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Landmark {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Landmark> Ptr;

        Landmark() {}

    private:
        
};

TRIAL_SLAM_NAMESPACE_END

#endif
