#pragma once
#ifndef TRIALSLAM_BACKEND_H_
#define TRIALSLAM_BACKEND_H_

#include "trialSlam/common.h"
#include "trialSlam/camera.h"
#include "trialSlam/map.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Backend {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        // Constructors
            Backend () {}

        // Getters & Setters
            void setCamera(std::shared_ptr<MonoCamera> camera) { _camera = camera; }

            void setMap(std::shared_ptr<Map> map) { _map = map; }

    private:
        std::shared_ptr<MonoCamera> _camera = nullptr;
        std::shared_ptr<Map> _map = nullptr;
};

TRIAL_SLAM_NAMESPACE_END

#endif
