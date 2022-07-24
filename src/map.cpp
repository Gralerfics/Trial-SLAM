#include "trialSlam/map.h"

TRIAL_SLAM_NAMESPACE_BEGIN

bool Map::addKeyFrame(std::shared_ptr<Frame> keyframe) {
    std::unique_lock<std::mutex> ulck(_data_mutex);
    _keyframes.insert(keyframe);
    if (_active_keyframes.size() >= _num_active_keyframes) {
        popActiveKeyFrame(keyframe);
        _active_keyframes.insert(keyframe);
    }
    return true;
}

bool Map::addLandmark(std::shared_ptr<Landmark> landmark) {
    std::unique_lock<std::mutex> ulck(_data_mutex);
    _landmarks.insert(landmark);
    _active_landmarks.insert(landmark);
    return true;
}

void Map::popActiveKeyFrame(std::shared_ptr<Frame> ref_keyframe) {
    // std::unique_lock<std::mutex> ulck(_data_mutex);
    // BTW pop the inactive landmarks.
    
}

TRIAL_SLAM_NAMESPACE_END
