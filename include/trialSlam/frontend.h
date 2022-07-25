#pragma once
#ifndef _TRIALSLAM_FRONTEND_H_
#define _TRIALSLAM_FRONTEND_H_

#include "trialSlam/common.h"
#include "trialSlam/camera.h"
#include "trialSlam/frame.h"
#include "trialSlam/map.h"

TRIAL_SLAM_NAMESPACE_BEGIN

enum class FrontendStatus {
    INIT_TRACK_FIRST,
    INIT_TRACK_SECOND,
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

        bool addFrame(Frame::Ptr frame);

        void initTrackFirst();

        void initTrackSecond();

        void track();

        void reset();

        int extractFrameFeatures(Frame::Ptr cur_frame);

        int trackFrameFeaturesFromTo(Frame::Ptr ref_frame, Frame::Ptr cur_frame);

        bool buildMapByInit(Frame::Ptr frame_first, Frame::Ptr frame_second);

        // bool addCurAsKeyFrame();

        // void estimateCurPose();

        Camera::Ptr getCamera() const { return _camera; }

        void setCamera(Camera::Ptr camera) { _camera = camera; }

        void setMap(Map::Ptr map) { _map = map; }

    private:
        Camera::Ptr _camera = nullptr;
        Frame::Ptr _cur_frame = nullptr, _last_frame = nullptr;
        Frame::Ptr _init_frame = nullptr;
        Map::Ptr _map = nullptr;
        FrontendStatus _status = FrontendStatus::INIT_TRACK_FIRST;

    public:
        cv::Ptr<cv::GFTTDetector> _features_detector;
        double _fd_quality_level = 0.01;
        double _fd_min_distance = 20;
        int _num_features = 150;
        int _num_features_for_init = 80;
        int _num_features_for_keyframe = 60;
};

TRIAL_SLAM_NAMESPACE_END

#endif
