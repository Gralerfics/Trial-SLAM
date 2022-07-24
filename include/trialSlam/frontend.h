#pragma once
#ifndef TRIALSLAM_FRONTEND_H_
#define TRIALSLAM_FRONTEND_H_

#include "trialSlam/common.h"
#include "trialSlam/camera.h"
#include "trialSlam/backend.h"
#include "trialSlam/map.h"
#include "trialSlam/dashboard.h"

TRIAL_SLAM_NAMESPACE_BEGIN

enum class FrontendStatus { INITA, INITB, TRACKING, LOST };

class Frontend {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        // Constructors
            Frontend () {}

        // Getters & Setters
            void setCamera(std::shared_ptr<MonoCamera> camera) { _camera = camera; }

            void setBackend(std::shared_ptr<Backend> backend) { _backend = backend; }

            void setMap(std::shared_ptr<Map> map) { _map = map; }

            void setDashboard(std::shared_ptr<Dashboard> dashboard) { _dashboard = dashboard; }

        // Actions
            bool addFrame(std::shared_ptr<Frame> frame);

    private:
        FrontendStatus _status = FrontendStatus::INITA;

        std::shared_ptr<MonoCamera> _camera = nullptr;
        std::shared_ptr<Backend> _backend = nullptr;
        std::shared_ptr<Map> _map = nullptr;
        std::shared_ptr<Dashboard> _dashboard = nullptr;
};

TRIAL_SLAM_NAMESPACE_END

#endif
