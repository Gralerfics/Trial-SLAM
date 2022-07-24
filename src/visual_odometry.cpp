#include "trialSlam/visual_odometry.h"

TRIAL_SLAM_NAMESPACE_BEGIN

bool VisualOdometry::initialize() {
    if (_frontend == nullptr) {
        return false;
    }

    return true;
}

void VisualOdometry::execute() {
    
}

TRIAL_SLAM_NAMESPACE_END
