#pragma once
#ifndef _TRIALSLAM_FRAME_H_
#define _TRIALSLAM_FRAME_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

        Frame() {}

    private:
        
};

TRIAL_SLAM_NAMESPACE_END

#endif
