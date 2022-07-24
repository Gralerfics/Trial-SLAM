#pragma once
#ifndef TRIALSLAM_VO_H_
#define TRIALSLAM_VO_H_

#include "trialSlam/common.h"
#include "trialSlam/camera.h"
#include "trialSlam/frontend.h"
#include "trialSlam/backend.h"
#include "trialSlam/map.h"
#include "trialSlam/dashboard.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class MonoVisualOdometry {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        // Constructors
            MonoVisualOdometry() {}
            
        // Getters & Setters
            std::shared_ptr<MonoCamera> getCameraPtr() const { return _camera; }

            MonoCamera& getCamera() const { return *_camera.get(); }

            void setCamera(std::shared_ptr<MonoCamera> camera) { _camera = camera; }

            void setFrontend(std::shared_ptr<Frontend> frontend) { _frontend = frontend; }

            void setBackend(std::shared_ptr<Backend> backend) { _backend = backend; }

            void setMap(std::shared_ptr<Map> map) { _map = map; }

            void setDashboard(std::shared_ptr<Dashboard> dashboard) { _dashboard = dashboard; }

        // Actions
            bool initialize();

            void execute();

            void stop();

    private:
        std::shared_ptr<MonoCamera> _camera = nullptr;
        std::shared_ptr<Frontend> _frontend = nullptr;
        std::shared_ptr<Backend> _backend = nullptr;
        std::shared_ptr<Map> _map = nullptr;
        std::shared_ptr<Dashboard> _dashboard = nullptr;
};

TRIAL_SLAM_NAMESPACE_END

#endif
