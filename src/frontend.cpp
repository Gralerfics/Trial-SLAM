#include "trialSlam/frontend.h"

TRIAL_SLAM_NAMESPACE_BEGIN

bool Frontend::initialize() {
    _status = FrontendStatus::INIT_TRACK_FIRST;

    _features_detector = cv::SiftFeatureDetector::create(_num_features);
    // _features_detector = cv::GFTTDetector::create(_num_features, _fd_quality_level, _fd_min_distance);

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

    if (_status == FrontendStatus::INIT_TRACK_FIRST) {
        initTrackFirst();
    } else if (_status == FrontendStatus::INIT_TRACK_SECOND) {
        initTrackSecond();
    } else if (_status == FrontendStatus::TRACKING) {
        track();
    } else if (_status == FrontendStatus::LOST) {
        reset();
    } else if (_status == FrontendStatus::SHUTDOWN) {
        return false;
    }

    // Temporary Dashboard
        cv::Mat _tmp(_cur_frame -> _img_raw);
        for (auto feature : _cur_frame -> getFeaturesRef()) {
            if (!feature) continue;
            cv::circle(_tmp, feature -> getKeyPoint().pt, 3, CV_RGB(255, 0, 0));
        }
        cv::imshow("Camera", _tmp);

    _last_frame = _cur_frame;

    return true;
}

// Temporary Dashboard
    static bool _showed_message = false;

void Frontend::initTrackFirst() {
    // Temporary Dashboard
        if (!_showed_message) std::cout << "The initial pose (Press enter after finished) ..." << std::endl;
        _showed_message = true;
        if (cv::waitKey(1) != 13) return;
        _showed_message = false;

    _last_keyframe = _cur_frame;

    int _num_extracted = extractFrameFeatures(_last_keyframe);
    std::cout << "Extracted: " << _num_extracted << std::endl;
    if (_num_extracted < _num_features_for_initializing) return;

    _status = FrontendStatus::INIT_TRACK_SECOND;
}

void Frontend::initTrackSecond() {
    // Temporary Dashboard
        if (!_showed_message) std::cout << "The corresponding pose (Press enter after finished) ..." << std::endl;
        _showed_message = true;
        if (cv::waitKey(1) != 13) return;
        _showed_message = false;

    int _num_effective = trackFrameFeaturesFromTo(_last_keyframe, _cur_frame);
    std::cout << "Successfully tracked: " << _num_effective << std::endl;
    if (_num_effective < _num_features_for_initializing) return;

    // Temporary Dashboard
        // cv::Mat _tmp;
        // std::vector<cv::DMatch> _matches;
        // std::vector<cv::KeyPoint> _pts_1, _pts_2;
        // int _match_cnt = 0;
        // for (int i = 0; i < _last_keyframe -> getFeaturesRef().size(); i ++) {
        //     if (_cur_frame -> getFeaturesRef()[i] != nullptr) {
        //         _matches.push_back(cv::DMatch(_match_cnt, _match_cnt, 0));
        //         _pts_1.push_back(_last_keyframe -> getFeaturesRef()[i] -> getKeyPoint());
        //         _pts_2.push_back(_cur_frame -> getFeaturesRef()[i] -> getKeyPoint());
        //         _match_cnt ++;
        //     }
        // }
        // cv::drawMatches(
        //     _last_keyframe -> _img_raw, _pts_1,
        //     _cur_frame -> _img_raw, _pts_2,
        //     _matches, _tmp
        // );
        // cv::imshow("Initial Matches", _tmp);
        // cv::waitKey(0);
        // cv::destroyWindow("Initial Matches");

    if (buildMapByEpipolarAndTriangulation(_last_keyframe, _cur_frame, false)) {
        _status = FrontendStatus::TRACKING;
        if (_dashboard) {
            _dashboard -> setCurrentFrame(_cur_frame);
            _dashboard -> update();
        }
    } else {
        _status = FrontendStatus::LOST;
    }
}

void Frontend::track() {
    std::cout << "Current frame (#" << _cur_frame -> getId() << "): " << std::endl;

    if (_last_frame)
        _cur_frame -> setTcw(_relative_move * _last_frame -> getTcw());

    int _num_effective = trackFrameFeaturesFromTo(_last_frame, _cur_frame);
    std::cout << "\t_num_effective = " << _num_effective << std::endl;
    if (_num_effective < _num_features_for_tracking) {
        _status = FrontendStatus::LOST;
        return;
    }

    int _num_included = estimatePosePnP(_cur_frame);
    std::cout << "\t_num_included = " << _num_included << std::endl;
    // if (_num_included < _num_features_for_tracking) {
    //     _status = FrontendStatus::LOST;
    //     return;
    // }
    
    // landmarks for pose estimation decrease to a low level
    if (_num_included < _num_features_for_keyframe) {
        if (!addFrameAsKeyFrame(_cur_frame)) {
            _status = FrontendStatus::LOST;
            return;
        }
    }

    _relative_move = _cur_frame -> getTcw() * _last_frame -> getTcw().inverse();

    if (_dashboard)
        _dashboard -> setCurrentFrame(_cur_frame);
}

void Frontend::reset() {
    // TODO: monocamera troubles ...
    std::cout << "Track lost (Press any key to quit) ..." << std::endl;
    cv::waitKey(0);
    _status = FrontendStatus::SHUTDOWN;
}

int Frontend::extractFrameFeatures(Frame::Ptr cur_frame) {
    // Exclude the existed feature positions.
    cv::Mat _mask(cur_frame -> _img_raw.size(), CV_8UC1, 255);
    for (auto& feature : cur_frame -> getFeaturesRef()) {
        cv::KeyPoint _kp = feature -> getKeyPoint();
        cv::rectangle(_mask, _kp.pt - cv::Point2f(10, 10), _kp.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> _keypoints;
    _features_detector -> detect(cur_frame -> _img_raw, _keypoints, _mask);
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
    std::vector<unsigned long> _feature_index;
    unsigned long _i = 0;
    for (auto& feature : ref_frame -> getFeaturesRef()) {
        if (feature) {
            auto landmark = feature -> getLandmark().lock();
            _pts_ref.push_back(feature -> getKeyPoint().pt);
            if (landmark) {
                auto proj = _camera -> world2pixel(landmark -> getPosition(), _cur_frame -> getTcw());
                _pts_cur.push_back(cv::Point2f(proj[0], proj[1]));
                _feature_index.push_back(_i);
            } else {
                _pts_cur.push_back(feature -> getKeyPoint().pt);
                _feature_index.push_back(_i);
            }
        }
        _i ++;
    }
    
    std::vector<uchar> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(
        ref_frame -> _img_raw, cur_frame -> _img_raw,
        _pts_ref, _pts_cur,
        status, error, cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW
    );

    int _cnt = 0;
    for (int i = 0; i < status.size(); i ++) {
        Feature::Ptr feature = nullptr;
        if (status[i]) {
            feature = std::make_shared<Feature>(cur_frame, cv::KeyPoint(_pts_cur[i], 6));
            Landmark::Ptr landmark = ref_frame -> getFeaturesRef()[_feature_index[i]] -> getLandmark().lock();
            if (landmark) feature -> setLandmark(landmark);
            _cnt ++;
        }
        cur_frame -> getFeaturesRef().push_back(feature);
    }

    return _cnt;
}

bool Frontend::buildMapByEpipolarAndTriangulation(Frame::Ptr frame_first, Frame::Ptr frame_second, bool isT2Known) {
    std::vector<cv::Point2f> _pts_1, _pts_2; // P_uv
    std::vector<cv::Point2f> _pts_1_c, _pts_2_c; // P_c
    std::vector<unsigned long> _feature_index;
    for (unsigned long i = 0; i < frame_first -> getFeaturesRef().size(); i ++) {
        if (frame_second -> getFeaturesRef()[i] != nullptr) {
            cv::Point2f _pt_1 = frame_first -> getFeaturesRef()[i] -> getKeyPoint().pt;
            cv::Point2f _pt_2 = frame_second -> getFeaturesRef()[i] -> getKeyPoint().pt;
            if (!isT2Known) {
                _pts_1.push_back(_pt_1);
                _pts_2.push_back(_pt_2);
            }
            Vec3 _pt_1_c = _camera -> pixel2camera(Vec2(_pt_1.x, _pt_1.y));
            Vec3 _pt_2_c = _camera -> pixel2camera(Vec2(_pt_2.x, _pt_2.y));
            _pts_1_c.push_back(cv::Point2f(_pt_1_c.x(), _pt_1_c.y()));
            _pts_2_c.push_back(cv::Point2f(_pt_2_c.x(), _pt_2_c.y()));
            _feature_index.push_back(i);
        }
    }
    
    Mat4x4 _T_1 = frame_first -> getTcw().matrix();
    cv::Mat T_1 = (cv::Mat_<float>(3, 4) <<
        _T_1(0, 0), _T_1(0, 1), _T_1(0, 2), _T_1(0, 3),
        _T_1(1, 0), _T_1(1, 1), _T_1(1, 2), _T_1(1, 3),
        _T_1(2, 0), _T_1(2, 1), _T_1(2, 2), _T_1(2, 3)
    );
    
    cv::Mat T_2;
    if (!isT2Known) {
        cv::Mat R, t;
        cv::Mat _essential_matrix = cv::findEssentialMat(_pts_1, _pts_2, _camera -> getK_CV(), cv::RANSAC);
        cv::recoverPose(_essential_matrix, _pts_1, _pts_2, _camera -> getK_CV(), R, t);
        T_2 = (cv::Mat_<float>(3, 4) <<
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
        );

        Mat3x3 R_Eigen;
        Vec3 t_Eigen;
        cv::cv2eigen(R, R_Eigen);
        cv::cv2eigen(t, t_Eigen);
        frame_second -> setTcw(SE3(R_Eigen, t_Eigen));
    } else {
        Mat4x4 _T_2 = frame_second -> getTcw().matrix();
        T_2 = (cv::Mat_<float>(3, 4) <<
            _T_2(0, 0), _T_2(0, 1), _T_2(0, 2), _T_2(0, 3),
            _T_2(1, 0), _T_2(1, 1), _T_2(1, 2), _T_2(1, 3),
            _T_2(2, 0), _T_2(2, 1), _T_2(2, 2), _T_2(2, 3)
        );
    }
    
    cv::Mat _pts_hw;
    cv::triangulatePoints(T_1, T_2, _pts_1_c, _pts_2_c, _pts_hw);
    for (int i = 0; i < _pts_hw.cols; i ++) {
        cv::Mat _p_hw = _pts_hw.col(i);
        _p_hw /= _p_hw.at<float>(3, 0);
        Vec3 p_w(_p_hw.at<float>(0, 0), _p_hw.at<float>(1, 0), _p_hw.at<float>(2, 0));

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
    
    if (!isT2Known) {
        frame_first -> markAsKeyFrame();
        _map -> addKeyFrame(frame_first);
    }
    frame_second -> markAsKeyFrame();
    _map -> addKeyFrame(frame_second);

    _last_keyframe = frame_second;

    // TODO: Backend update

    return true;
}

bool Frontend::addFrameAsKeyFrame(Frame::Ptr cur_frame) {
    std::cout << "Adding new keyframe ..." << std::endl;
    Frame::Ptr ref_frame = _last_keyframe;

    // extract new features
    ref_frame -> getFeaturesRef().clear(); // temporary solution, TODO: DMatch
    int _num_extracted = extractFrameFeatures(ref_frame);
    std::cout << "Extracted: " << _num_extracted << std::endl;
    if (_num_extracted < _num_features_for_keyframe) return false;

    // track
    cur_frame -> getFeaturesRef().clear(); // temporary solution, TODO: DMatch
    int _num_effective = trackFrameFeaturesFromTo(ref_frame, cur_frame);
    std::cout << "Successfully tracked: " << _num_effective << std::endl;
    if (_num_effective < _num_features_for_keyframe) return false;

    // build map
    if (!buildMapByEpipolarAndTriangulation(ref_frame, cur_frame, true)) {
        return false;
    }

    // mark
    cur_frame -> markAsKeyFrame();
    _map -> addKeyFrame(cur_frame);

    // dashboard
    if (_dashboard) {
        _dashboard -> setCurrentFrame(cur_frame);
        _dashboard -> update();
    }

    // TODO: Backend update

    return true;
}

cv::Point2d pixelToNormalizedCam(const cv::Point2d& p, const cv::Mat& K) {
    return cv::Point2d(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),            // u = fx * (X' / Z') + cx  => (X' / Z') = (u - cx) / fx
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)             // v = ...                  => (Y' / Z') = ...
    );
}

struct SnavelyReprojectionError {
    cv::Point2d _u;
    cv::Point3d _P;
    cv::Mat _K;

    SnavelyReprojectionError(cv::Point2d u, cv::Point3d P, cv::Mat K): _u(u), _P(P), _K(K) {}

    template<typename T> bool operator() (const T* const ksi, T* residual) const {      // ksi = [rho(平移) phi(旋转)]T
        T P[3], P_prime[3];
        P_prime[0] = (T) _P.x, P_prime[1] = (T) _P.y, P_prime[2] = (T) _P.z;

        ceres::AngleAxisRotatePoint(ksi + 3, P_prime, P);
        P[0] += ksi[0], P[1] += ksi[1], P[2] += ksi[2];

        P[0] /= P[2], P[1] /= P[2];

        residual[0] = (T) _u.x - (_K.at<double>(0, 0) * P[0] + _K.at<double>(0, 2));
        residual[1] = (T) _u.y - (_K.at<double>(1, 1) * P[1] + _K.at<double>(1, 2));

        return true;
    }

    static ceres::CostFunction* Create(cv::Point2d u, cv::Point3d P, cv::Mat K) {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6>(
            new SnavelyReprojectionError(u, P, K)
        ));
    }
};

int Frontend::estimatePosePnP(Frame::Ptr cur_frame) {
/*
    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;
    int _cnt = 0;
    for (unsigned long i = 0; i < cur_frame -> getFeaturesRef().size(); i ++) {
        auto feature = cur_frame -> getFeaturesRef()[i];
        if (!feature) continue;
        auto landmark = feature -> getLandmark().lock();
        if (!landmark) continue;
        _cnt ++;
        Vec3 _p = landmark -> getPosition();
        pts_3d.push_back(cv::Point3f(_p.x(), _p.y(), _p.z()));
        pts_2d.push_back(feature -> getKeyPoint().pt);
    }

    double ksi[6] = {0};

    ceres::Problem problem;
    for (int i = 0; i < pts_3d.size(); i ++) {
        ceres::CostFunction* costFunction = SnavelyReprojectionError::Create(pts_2d[i], pts_3d[i], _camera -> getK_CV());
        problem.AddResidualBlock(
            costFunction,
            nullptr,
            ksi
        );
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << std::endl;

    cv::Mat r = (cv::Mat_<double>(3, 1) << ksi[3], ksi[4], ksi[5]);
    cv::Mat t = (cv::Mat_<double>(3, 1) << ksi[0], ksi[1], ksi[2]);

    cv::Mat R;
    Rodrigues(r, R);

    Mat3x3 R_Eigen;
    Vec3 t_Eigen;
    cv::cv2eigen(R, R_Eigen);
    cv::cv2eigen(t, t_Eigen);

    cur_frame -> setTcw(SE3(R_Eigen, t_Eigen));

    return _cnt;
*/

// /*
    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;
    int _cnt = 0;
    for (unsigned long i = 0; i < cur_frame -> getFeaturesRef().size(); i ++) {
        auto feature = cur_frame -> getFeaturesRef()[i];
        if (!feature) continue;
        auto landmark = feature -> getLandmark().lock();
        if (!landmark) continue;
        _cnt ++;
        Vec3 _p = landmark -> getPosition();
        pts_3d.push_back(cv::Point3f(_p.x(), _p.y(), _p.z()));
        pts_2d.push_back(feature -> getKeyPoint().pt);
    }

    cv::Mat r, t, R;
    cv::solvePnP(pts_3d, pts_2d, _camera -> getK_CV(), cv::Mat(), r, t, false);
    cv::Rodrigues(r, R);

    Mat3x3 R_Eigen;
    Vec3 t_Eigen;
    cv::cv2eigen(R, R_Eigen);
    cv::cv2eigen(t, t_Eigen);

    cur_frame -> setTcw(SE3(R_Eigen, t_Eigen));

    return _cnt;
// */

/*
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // Vertex
    poseVertex* vertex_p = new poseVertex();
    vertex_p -> setId(0);
    // vertex_p -> setEstimate(cur_frame -> getTcw());
    optimizer.addVertex(vertex_p);

    // Edge
    int edge_index = 1;
    std::vector<projectionPoseEdge*> edges;
    std::vector<Feature::Ptr> features;
    for (unsigned long i = 0; i < cur_frame -> getFeaturesRef().size(); i ++) {
        auto feature = cur_frame -> getFeaturesRef()[i];
        if (!feature) continue;
        auto landmark = feature -> getLandmark().lock();
        if (!landmark) continue;
        features.push_back(cur_frame -> getFeaturesRef()[i]);
        projectionPoseEdge* edge = new projectionPoseEdge(landmark -> getPosition(), _camera -> getK_Eigen());
        edge -> setId(edge_index);
        edge_index ++;
        edge -> setVertex(0, vertex_p);
        edge -> setMeasurement(Vec2(feature -> getKeyPoint().pt.x, feature -> getKeyPoint().pt.y));
        edge -> setInformation(Mat2x2::Identity());
        edge -> setRobustKernel(new g2o::RobustKernelHuber());
        edges.push_back(edge);
        optimizer.addEdge(edge);
    }

    const double khi2_thresold = 5.991;
    int _cnt_excluded;
    // for (int it = 0; it < 4; it ++) {
        // vertex_p -> setEstimate(cur_frame -> getTcw());
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        _cnt_excluded = 0;

        for (unsigned long i = 0; i < edges.size(); i ++) {
            auto edge = edges[i];
            if (features[i] -> isExcluded()) edge -> computeError();
            if (edge -> chi2() > khi2_thresold) {
                features[i] -> exclude();
                edge -> setLevel(1);
                _cnt_excluded ++;
            } else {
                features[i] -> include();
                edge -> setLevel(0);
            }

            // if (it == 2) {
            //     edge -> setRobustKernel(nullptr);
            // }
        }
    // }

    // update pose
    cur_frame -> setTcw(vertex_p -> estimate());

    // update features
    for (auto& feature : features) {
        if (feature -> isExcluded()) {
            feature -> getLandmark().reset();
            feature -> include(); // ?
        }
    }

    return features.size() - _cnt_excluded;
*/
}

TRIAL_SLAM_NAMESPACE_END
