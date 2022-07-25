#pragma once
#ifndef _TRIALSLAM_DASHBOARD_H_
#define _TRIALSLAM_DASHBOARD_H_

#include "trialSlam/common.h"
#include "trialSlam/frame.h"
#include "trialSlam/map.h"
#include "trialSlam/camera.h"

#include <pangolin/pangolin.h>

TRIAL_SLAM_NAMESPACE_BEGIN

class Dashboard {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Dashboard> Ptr;

        Dashboard() {}

        bool initialize();

        void stop();

        void loop();

        void update();

        void drawFrame(Frame::Ptr frame, const float* color);

        void drawLandmarks();

        void moveToCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

        void setMap(Map::Ptr map) { _map = map; }

        void setCamera(Camera::Ptr camera) { _camera = camera; }

        void setCurrentFrame(Frame::Ptr cur_frame) {
            std::unique_lock<std::mutex> ulock(_mutex);
            _cur_frame = cur_frame;
        }

    private:
        Map::Ptr _map = nullptr;
        Camera::Ptr _camera = nullptr;
        Frame::Ptr _cur_frame = nullptr;

        Map::KeyFramesMap _active_keyframes;
        Map::LandmarksMap _active_landmarks;

        std::thread _dashboard_thread;
        bool _dashboard_running = false;

        std::mutex _mutex;
};

TRIAL_SLAM_NAMESPACE_END

#endif
