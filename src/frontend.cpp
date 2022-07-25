#include "trialSlam/frontend.h"

TRIAL_SLAM_NAMESPACE_BEGIN

bool Frontend::initialize() {
    _status = FrontendStatus::INIT_TRACK_FIRST;

    _features_detector = cv::GFTTDetector::create(_num_features, _fd_quality_level, _fd_min_distance);

    if (_camera == nullptr || _map == nullptr) return false;

    if (!_camera -> open()) return false;

    return true;
}

void Frontend::stop() {
    // Temporary Dashboard
        cv::destroyWindow("Camera");

    _camera -> close();
}

bool Frontend::addFrame(Frame::Ptr frame) {
    _cur_frame = frame;

    // Temporary Dashboard
        cv::imshow("Camera", _cur_frame -> _img);

    if (_status == FrontendStatus::INIT_TRACK_FIRST) {
        initTrackFirst();
    } else if (_status == FrontendStatus::INIT_TRACK_SECOND) {
        initTrackSecond();
    } else if (_status == FrontendStatus::TRACKING) {
        track();
    } else if (_status == FrontendStatus::LOST) {
        reset();
    }

    _last_frame = _cur_frame;

    return true;
}

void Frontend::initTrackFirst() {
    // Temporary Dashboard
        std::cout << "The initial pose (Press enter after finished) ..." << std::endl;
        if (cv::waitKey(1) != 13) return;

    _init_frame = _cur_frame;
    
    int _num_extracted = extractFrameFeatures(_init_frame);
    std::cout << "Extracted: " << _num_extracted << std::endl;
    if (_num_extracted < _num_features_for_init) return;

    _status = FrontendStatus::INIT_TRACK_SECOND;
}

void Frontend::initTrackSecond() {
    // Temporary Dashboard
        std::cout << "The corresponding pose (Press enter after finished) ..." << std::endl;
        if (cv::waitKey(1) != 13) return;

    int _num_effective = trackFrameFeaturesFromTo(_init_frame, _cur_frame);
    std::cout << "Successfully tracked: " << _num_effective << std::endl;
    if (_num_effective < _num_features_for_init) return;

    // Temporary Dashboard
        cv::Mat _tmp;
        std::vector<cv::DMatch> _matches;
        std::vector<cv::KeyPoint> _pts_1, _pts_2;
        int _match_cnt = 0;
        for (int i = 0; i < _init_frame -> getFeaturesRef().size(); i ++) {
            if (_cur_frame -> getFeaturesRef()[i] != nullptr) {
                _matches.push_back(cv::DMatch(_match_cnt, _match_cnt, 0));
                _pts_1.push_back(_init_frame -> getFeaturesRef()[i] -> getKeyPoint());
                _pts_2.push_back(_cur_frame -> getFeaturesRef()[i] -> getKeyPoint());
                _match_cnt ++;
            }
        }
        cv::drawMatches(
            _init_frame -> _img_raw, _pts_1,
            _cur_frame -> _img_raw, _pts_2,
            _matches, _tmp
        );
        cv::imshow("Initial Matches", _tmp);
        cv::waitKey(0);
        cv::destroyWindow("Initial Matches");

    if (buildMapByInit(_init_frame, _cur_frame)) {
        _status = FrontendStatus::TRACKING;
        // TODO: Dashboard
    } else {
        _status = FrontendStatus::INIT_TRACK_FIRST;
    }
}

void Frontend::track() {

}

void Frontend::reset() {
    // TODO
}

int Frontend::extractFrameFeatures(Frame::Ptr cur_frame) {
    // Exclude the existed feature positions.
    cv::Mat _mask(cur_frame -> _img.size(), CV_8UC1, 255);
    for (auto& feature : cur_frame -> getFeaturesRef()) {
        cv::KeyPoint _kp = feature -> getKeyPoint();
        cv::rectangle(_mask, _kp.pt - cv::Point2f(10, 10), _kp.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> _keypoints;
    _features_detector -> detect(cur_frame -> _img, _keypoints, _mask);
    int _cnt = 0;
    for (auto& keypoint : _keypoints) {
        cur_frame -> getFeaturesRef().push_back(Feature::Ptr(new Feature(cur_frame, keypoint)));
        _cnt ++;
    }

    return _cnt;
}

int Frontend::trackFrameFeaturesFromTo(Frame::Ptr ref_frame, Frame::Ptr cur_frame) {
    // TODO: Initial guess
    std::vector<cv::Point2f> _pts_ref, _pts_cur;
    for (auto& keypoint : ref_frame -> getFeaturesRef())
        _pts_ref.push_back(keypoint -> getKeyPoint().pt);
    
    std::vector<uchar> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(
        ref_frame -> _img, cur_frame -> _img,
        _pts_ref, _pts_cur,
        status, error // , cv::Size(11, 11)
    );

    int _cnt = 0;
    for (int i = 0; i < status.size(); i ++) {
        Feature::Ptr feature = nullptr;
        if (status[i]) {
            feature = std::make_shared<Feature>(cur_frame, cv::KeyPoint(_pts_cur[i], 6));
            _cnt ++;
        }
        cur_frame -> getFeaturesRef().push_back(feature);
    }

    return _cnt;
}

bool Frontend::buildMapByInit(Frame::Ptr frame_first, Frame::Ptr frame_second) {
    
    return true;
}

TRIAL_SLAM_NAMESPACE_END
