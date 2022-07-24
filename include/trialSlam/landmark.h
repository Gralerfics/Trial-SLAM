#pragma once
#ifndef TRIALSLAM_LANDMARK_H_
#define TRIALSLAM_LANDMARK_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Landmark {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // Constructors
            Landmark() {}

            Landmark(Vec3& position): _position(position) {}

        // Getters & Setters
            Vec3 getPosition() {
                std::unique_lock<std::mutex> ulck(_data_mutex);
                return _position;
            }

            void setPosition(const Vec3& position) {
                std::unique_lock<std::mutex> ulck(_data_mutex);
                _position = position;
            }

            int getActiveObservatorFrameNum() {
                std::unique_lock<std::mutex> ulck(_data_mutex);
                return _num_active_observator_frame;
            }

            void addActiveObservatorFrameNum() {
                std::unique_lock<std::mutex> ulck(_data_mutex);
                _num_active_observator_frame ++;
            }

            void decActiveObservatorFrameNum() {
                std::unique_lock<std::mutex> ulck(_data_mutex);
                _num_active_observator_frame --;
            }

    private:
        Vec3 _position = Vec3::Zero();
        int _num_active_observator_frame = 0;
        std::mutex _data_mutex;
};

TRIAL_SLAM_NAMESPACE_END

#endif
