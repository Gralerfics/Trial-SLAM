#pragma once
#ifndef TRIALSLAM_MAP_H_
#define TRIALSLAM_MAP_H_

#include "trialSlam/common.h"
#include "trialSlam/landmark.h"
#include "trialSlam/frame.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        // Constructors
            Map () {}

        // Getters & Setters
            std::unordered_set<std::shared_ptr<Landmark>> getLandmarks() {
                std::unique_lock<std::mutex> ulck(_data_mutex);
                return _landmarks;
            }

            std::unordered_set<std::shared_ptr<Landmark>> getActiveLandmarks() {
                std::unique_lock<std::mutex> ulck(_data_mutex);
                return _active_landmarks;
            }

            std::unordered_set<std::shared_ptr<Frame>> getFrames() {
                std::unique_lock<std::mutex> ulck(_data_mutex);
                return _keyframes;
            }

            std::unordered_set<std::shared_ptr<Frame>> getActiveFrames() {
                std::unique_lock<std::mutex> ulck(_data_mutex);
                return _active_keyframes;
            }

        // Actions
            bool addKeyFrame(std::shared_ptr<Frame> keyframe);

            bool addLandmark(std::shared_ptr<Landmark> landmark);

            void popActiveKeyFrame(std::shared_ptr<Frame> ref_keyframe);

    private:
        const int _num_active_keyframes = 6;

        std::unordered_set<std::shared_ptr<Landmark>> _landmarks, _active_landmarks;
        std::unordered_set<std::shared_ptr<Frame>> _keyframes, _active_keyframes;
        std::mutex _data_mutex;
};

TRIAL_SLAM_NAMESPACE_END

#endif
