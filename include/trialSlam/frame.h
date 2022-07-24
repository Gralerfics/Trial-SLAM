#pragma once
#ifndef TRIALSLAM_FRAME_H_
#define TRIALSLAM_FRAME_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // Constructors
            Frame() {}

            Frame(const SE3& pose, const cv::Mat& img)
                : _pose(pose), _img(img) {
                    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch()
                    );
                    _time_stamp = ms.count();
                }

        // Getters & Setters
            SE3 getPose() {
                std::unique_lock<std::mutex> ulck(_data_mutex);
                return _pose;
            }

            void setPose(const SE3& pose) {
                std::unique_lock<std::mutex> ulck(_data_mutex);
                _pose = pose;
            }

    private:
        SE3 _pose;
        cv::Mat _img;
        unsigned long _time_stamp;
        std::mutex _data_mutex;
};

TRIAL_SLAM_NAMESPACE_END

#endif
