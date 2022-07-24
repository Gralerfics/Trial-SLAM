#include "trialSlam/frontend.h"

TRIAL_SLAM_NAMESPACE_BEGIN

bool Frontend::initialize() {
    _status = FrontendStatus::INIT_FIRST;

    if (_camera == nullptr || _map == nullptr) {
        return false;
    }

    if (!_camera -> open()) {
        return false;
    }

    return true;
}

void Frontend::stop() {
    _camera -> close();
}

bool Frontend::addFrame(Frame::Ptr frame) {
    // TODO
    return true;
}

TRIAL_SLAM_NAMESPACE_END
