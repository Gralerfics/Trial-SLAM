#pragma once
#ifndef _TRIALSLAM_MAP_H_
#define _TRIALSLAM_MAP_H_

#include "trialSlam/common.h"
#include "trialSlam/feature.h"
#include "trialSlam/frame.h"
#include "trialSlam/landmark.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyFramesMap;
        typedef std::unordered_map<unsigned long, Landmark::Ptr> LandmarksMap;

        Map() {}

        KeyFramesMap getKeyFrames() {
            std::unique_lock<std::mutex> ulock(_mutex);
            return _keyframes;
        }

        KeyFramesMap getActiveKeyFrames() {
            std::unique_lock<std::mutex> ulock(_mutex);
            return _active_keyframes;
        }

        LandmarksMap getLandmarks() {
            std::unique_lock<std::mutex> ulock(_mutex);
            return _landmarks;
        }

        LandmarksMap getActiveLandmarks() {
            std::unique_lock<std::mutex> ulock(_mutex);
            return _active_landmarks;
        }

        void addKeyFrame(Frame::Ptr frame);

        void addLandmark(Landmark::Ptr landmark);

        void filterActiveLandmarks();

        void filterActiveKeyFrame(Frame::Ptr ref_frame, double dist_thresold);

    private:
        KeyFramesMap _keyframes, _active_keyframes;
        LandmarksMap _landmarks, _active_landmarks;
        std::mutex _mutex;

    public:
        int _num_active_keyframes = 6;
};

TRIAL_SLAM_NAMESPACE_END

#endif
