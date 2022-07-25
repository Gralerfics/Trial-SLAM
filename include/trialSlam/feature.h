#pragma once
#ifndef _TRIALSLAM_FEATURE_H_
#define _TRIALSLAM_FEATURE_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Frame;

class Landmark;

class Feature {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Feature> Ptr;

        Feature() {}

        Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint& keypoint)
            : _frame(frame), _keypoint(keypoint) {}

        std::weak_ptr<Frame> getFrame() {
            return _frame;
        }

        void setFrame(std::shared_ptr<Frame> frame) {
            _frame = frame;
        }

        std::weak_ptr<Landmark> getLandmark() {
            return _landmark;
        }

        void setLandmark(std::shared_ptr<Landmark> landmark) {
            _landmark = landmark;
        }

        cv::KeyPoint getKeyPoint() {
            return _keypoint;
        }

        void setKeyPoint(cv::KeyPoint& keypoint) {
            _keypoint = keypoint;
        }

        bool isExcluded() {
            return _excluded;
        }

        void exclude() {
            _excluded = true;
        }

        void include() {
            _excluded = false;
        }

    private:
        std::weak_ptr<Frame> _frame;
        std::weak_ptr<Landmark> _landmark;
        cv::KeyPoint _keypoint;
        bool _excluded = false;
};

TRIAL_SLAM_NAMESPACE_END

#endif
