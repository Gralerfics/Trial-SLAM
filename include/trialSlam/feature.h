#pragma once
#ifndef _TRIALSLAM_FEATURE_H_
#define _TRIALSLAM_FEATURE_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Feature {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Feature> Ptr;

        Feature() {}

    private:
        
};

TRIAL_SLAM_NAMESPACE_END

#endif
