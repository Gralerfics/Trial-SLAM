#include "trialSlam/frontend.h"

TRIAL_SLAM_NAMESPACE_BEGIN

bool Frontend::initialize() {
    _status = FrontendStatus::INIT_TRACK_FIRST;

    _features_detector = cv::GFTTDetector::create(_num_features, _fd_quality_level, _fd_min_distance);

    if (_camera == nullptr || _map == nullptr) return false;

    if (!_camera -> open()) return false;

    _dashboard -> initialize();

    return true;
}

void Frontend::stop() {
    // Temporary Dashboard
        cv::destroyWindow("Camera");

    _camera -> close();
    _dashboard -> stop();
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
        // cv::Mat _tmp;
        // std::vector<cv::DMatch> _matches;
        // std::vector<cv::KeyPoint> _pts_1, _pts_2;
        // int _match_cnt = 0;
        // for (int i = 0; i < _init_frame -> getFeaturesRef().size(); i ++) {
        //     if (_cur_frame -> getFeaturesRef()[i] != nullptr) {
        //         _matches.push_back(cv::DMatch(_match_cnt, _match_cnt, 0));
        //         _pts_1.push_back(_init_frame -> getFeaturesRef()[i] -> getKeyPoint());
        //         _pts_2.push_back(_cur_frame -> getFeaturesRef()[i] -> getKeyPoint());
        //         _match_cnt ++;
        //     }
        // }
        // cv::drawMatches(
        //     _init_frame -> _img_raw, _pts_1,
        //     _cur_frame -> _img_raw, _pts_2,
        //     _matches, _tmp
        // );
        // cv::imshow("Initial Matches", _tmp);
        // cv::waitKey(0);
        // cv::destroyWindow("Initial Matches");

    if (buildMapByInit(_init_frame, _cur_frame)) {
        _status = FrontendStatus::TRACKING;
        if (_dashboard) {
            _dashboard -> setCurrentFrame(_cur_frame);
            _dashboard -> update();
        }
    } else {
        _status = FrontendStatus::INIT_TRACK_FIRST;
    }
}

void Frontend::track() {
    

    if (_dashboard)
        _dashboard -> setCurrentFrame(_cur_frame);
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
    std::vector<cv::Point2f> _pts_1, _pts_2; // P_uv
    std::vector<cv::Point2f> _pts_1_c, _pts_2_c; // P_c
    std::vector<int> _feature_index;
    for (int i = 0; i < frame_first -> getFeaturesRef().size(); i ++) {
        if (_cur_frame -> getFeaturesRef()[i] != nullptr) {
            cv::Point2f _pt_1 = frame_first -> getFeaturesRef()[i] -> getKeyPoint().pt;
            cv::Point2f _pt_2 = frame_second -> getFeaturesRef()[i] -> getKeyPoint().pt;
            _pts_1.push_back(_pt_1);
            _pts_2.push_back(_pt_2);
            Vec3 _pt_1_c = _camera -> pixel2camera(Vec2(_pt_1.x, _pt_1.y));
            Vec3 _pt_2_c = _camera -> pixel2camera(Vec2(_pt_2.x, _pt_2.y));
            _pts_1_c.push_back(cv::Point2f(_pt_1_c.x(), _pt_1_c.y()));
            _pts_2_c.push_back(cv::Point2f(_pt_2_c.x(), _pt_2_c.y()));
            _feature_index.push_back(i);
        }
    }

    cv::Mat R, t;
    cv::Mat _essential_matrix = cv::findEssentialMat(_pts_1, _pts_2, _camera -> getK_CV(), cv::RANSAC);
    cv::recoverPose(_essential_matrix, _pts_1, _pts_2, _camera -> getK_CV(), R, t);

    Mat3x3 R_Eigen;
    Vec3 t_Eigen;
    cv::cv2eigen(R, R_Eigen);
    cv::cv2eigen(t, t_Eigen);
    frame_first -> setTcw(SE3(Mat3x3::Identity(), Vec3::Zero()));
    frame_second -> setTcw(SE3(R_Eigen, t_Eigen));

    cv::Mat T_1 = (cv::Mat_<float>(3, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0
    );
    cv::Mat T_2 = (cv::Mat_<float>(3, 4) <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
    );

    cv::Mat _pts_hc;
    cv::triangulatePoints(T_1, T_2, _pts_1_c, _pts_2_c, _pts_hc);
    for (int i = 0; i < _pts_hc.cols; i ++) {
        cv::Mat _p_hc = _pts_hc.col(i);
        _p_hc /= _p_hc.at<float>(3, 0);
        // because T_1 = Zero, p_w = p_c_1
        Vec3 p_w(_p_hc.at<float>(0, 0), _p_hc.at<float>(1, 0), _p_hc.at<float>(2, 0));

        if (p_w.z() > 0) {
            Landmark::Ptr landmark = Landmark::Create();
            landmark -> setPosition(p_w);
            landmark -> addObservedBy(frame_first -> getFeaturesRef()[_feature_index[i]]);
            landmark -> addObservedBy(frame_second -> getFeaturesRef()[_feature_index[i]]);
            frame_first -> getFeaturesRef()[_feature_index[i]] -> setLandmark(landmark);
            frame_second -> getFeaturesRef()[_feature_index[i]] -> setLandmark(landmark);
            _map -> addLandmark(landmark);
        }
    }

    frame_first -> markAsKeyFrame();
    frame_second -> markAsKeyFrame();
    _map -> addKeyFrame(frame_first);
    _map -> addKeyFrame(frame_second);
    // TODO: Backend update

    return true;
}

TRIAL_SLAM_NAMESPACE_END
