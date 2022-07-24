#pragma once
#ifndef _TRIALSLAM_FRONTEND_H_
#define _TRIALSLAM_FRONTEND_H_

#include "trialSlam/common.h"
#include "trialSlam/camera.h"
#include "trialSlam/frame.h"
#include "trialSlam/map.h"

TRIAL_SLAM_NAMESPACE_BEGIN

enum class FrontendStatus {
    INIT_FIRST,
    INIT_SECOND,
    TRACKING,
    LOST
};

class Frontend {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frontend> Ptr;

        Frontend() {}

        bool initialize();

        void stop();

        // void reset();

        bool addFrame(Frame::Ptr frame);

        // bool addCurAsKeyFrame();

        // void track();

        // void extractFeatures();

        // void trackFeatures();

        // void estimateCurPose();

        Camera::Ptr getCamera() const { return _camera; }

        void setCamera(Camera::Ptr camera) { _camera = camera; }

        void setMap(Map::Ptr map) { _map = map; }

    private:
        Camera::Ptr _camera = nullptr;
        Frame::Ptr _cur_frame = nullptr, _last_frame = nullptr;
        Map::Ptr _map = nullptr;
        FrontendStatus _status = FrontendStatus::INIT_FIRST;

    public:
        int _num_features = 150;
        int _num_features_for_init = 80;
        int _num_features_for_keyframe = 60;
};

TRIAL_SLAM_NAMESPACE_END

#endif
