#pragma once
#ifndef TRIALSLAM_DASHBOARD_H_
#define TRIALSLAM_DASHBOARD_H_

#include "trialSlam/common.h"
#include "trialSlam/map.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Dashboard {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        // Constructors
            Dashboard() {}

        // Getters & Setters
            void setMap(std::shared_ptr<Map> map) { _map = map; }

    private:
        std::shared_ptr<Map> _map = nullptr;
};

TRIAL_SLAM_NAMESPACE_END

#endif
