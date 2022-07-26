#include "trialSlam/visual_odometry.h"
// #include "trialSlam/frame.h"

TRIAL_SLAM_NAMESPACE_BEGIN

bool VisualOdometry::initialize() {
    if (_frontend == nullptr) {
        return false;
    }

    if (!_frontend -> initialize()) {
        return false;
    }

    return true;
}

void VisualOdometry::execute() {
    cv::Mat shot;
    while (_frontend -> getCamera() -> capture(shot)) {
        Frame::Ptr frame = Frame::Create();

        shot.copyTo(frame -> _img_raw);
        cv::cvtColor(shot, frame -> _img, cv::COLOR_RGB2GRAY);
        
        if (!_frontend -> addFrame(frame)) break;
        
        if (cv::waitKey(1) == 27) break;
    }

    stop();
}

void VisualOdometry::stop() {
    _frontend -> stop();
}

TRIAL_SLAM_NAMESPACE_END
