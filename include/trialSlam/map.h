#pragma once
#ifndef _TRIALSLAM_MAP_H_
#define _TRIALSLAM_MAP_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;

        Map() {}

    private:
        
};

TRIAL_SLAM_NAMESPACE_END

#endif
