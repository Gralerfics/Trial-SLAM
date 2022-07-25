#pragma once
#ifndef _TRIALSLAM_FRAME_H_
#define _TRIALSLAM_FRAME_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Feature;

class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

        Frame() {}

        Frame(unsigned long id, const cv::Mat& img, const SE3& T_cw)
            : _id(id), _img(img), _T_cw(T_cw) {}

        unsigned long getId() {
            return _id;
        }

        unsigned long getKeyFrameId() {
            return _keyframe_id;
        }

        std::vector<std::shared_ptr<Feature>>& getFeaturesRef() {
            return _features;
        }

        bool isKeyFrame() {
            return _is_keyframe;
        }

        SE3 getTcw() {
            std::unique_lock<std::mutex> ulock(_mutex);
            return _T_cw;
        }

        void setTcw(const SE3& T_cw) {
            std::unique_lock<std::mutex> ulock(_mutex);
            _T_cw = T_cw;
        }

        bool markAsKeyFrame();

        static Frame::Ptr Create();

    private:
        std::vector<std::shared_ptr<Feature>> _features;
        unsigned long _id, _keyframe_id;
        bool _is_keyframe;
        SE3 _T_cw;
        std::mutex _mutex;                                      // for _T_cw.

        static unsigned long __id, __keyframe_id;

    public:
        cv::Mat _img, _img_raw;
};

TRIAL_SLAM_NAMESPACE_END

#endif
