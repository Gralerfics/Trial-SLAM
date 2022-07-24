#include "trialSlam/map.h"

TRIAL_SLAM_NAMESPACE_BEGIN

void Map::addKeyFrame(Frame::Ptr frame) {
    // if (_keyframes.find(frame -> getKeyFrameId()) == _keyframes.end()) {
        _keyframes.insert(std::make_pair(frame -> getKeyFrameId(), frame));
        _active_keyframes.insert(std::make_pair(frame -> getKeyFrameId(), frame));
    // } else {
    //     _keyframes[frame -> getKeyFrameId()] = frame;
    //     _active_keyframes[frame -> getKeyFrameId()] = frame;
    // }
    
    if (_active_keyframes.size() > _num_active_keyframes)
        filterActiveKeyFrame(frame, 0.2);
}

void Map::addLandmark(Landmark::Ptr landmark) {
    // if (_landmarks.find(landmark -> getId()) == _landmarks.end()) {
        _landmarks.insert(std::make_pair(landmark -> getId(), landmark));
        _active_landmarks.insert(std::make_pair(landmark -> getId(), landmark));
    // } else {
    //     _landmarks[landmark -> getId()] = landmark;
    //     _active_landmarks[landmark -> getId()] = landmark;
    // }
}

void Map::filterActiveLandmarks() {
    for (auto it = _active_landmarks.begin(); it != _active_landmarks.end();) {
        if (it -> second -> getNumObservedBys() <= 0) {
            it = _active_landmarks.erase(it);
        } else {
            it ++;
        }
    }
}

void Map::filterActiveKeyFrame(Frame::Ptr ref_frame, double dist_thresold) {
    double max_dist = -1, min_dist = 1e8;
    unsigned long max_index, min_index;
    SE3 T_wc = ref_frame -> getTcw().inverse();
    for (auto& act_frame : _active_keyframes) {
        if (act_frame.second != ref_frame) {
            double _dist = (act_frame.second -> getTcw() * T_wc).log().norm();
            if (_dist > max_dist) {
                max_dist = _dist;
                max_index = act_frame.first;
            }
            if (_dist < min_dist) {
                min_dist = _dist;
                min_index = act_frame.first;
            }
        }
    }

    Frame::Ptr rm_frame = _keyframes.at(min_dist < dist_thresold ? min_index : max_index);
    for (Feature::Ptr feature : rm_frame -> getFeatures()) {
        if (feature -> getLandmark().lock()) {
            feature -> getLandmark().lock() -> removeObservedBy(feature);
        }
    }

    filterActiveLandmarks();
}

TRIAL_SLAM_NAMESPACE_END
