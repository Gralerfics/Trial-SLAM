#include "trialSlam/frame.h"

TRIAL_SLAM_NAMESPACE_BEGIN

void Frame::addLandmark(std::shared_ptr<Landmark> landmark) {
    landmark -> addActiveObservatorFrameNum();
    _landmarks.insert(landmark);
}

void Frame::removeFromActiveKeyFrame() {
    std::unique_lock<std::mutex> ulck(_data_mutex);
    for (auto landmark : _landmarks) {
        landmark -> decActiveObservatorFrameNum();
        if (landmark -> getActiveObservatorFrameNum() <= 0) {
            // TODO
        }
    }
}

TRIAL_SLAM_NAMESPACE_END
