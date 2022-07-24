#include "trialSlam/vo.h"

TRIAL_SLAM_NAMESPACE_BEGIN

bool MonoVisualOdometry::initialize() {
    // Open the camera
    if (!_camera -> open()) {
        return false;
    }
    
    // Create the components
    _frontend = std::shared_ptr<Frontend>(new Frontend());
    _backend = std::shared_ptr<Backend>(new Backend());
    _map = std::shared_ptr<Map>(new Map());
    _dashboard = std::shared_ptr<Dashboard>(new Dashboard());

    // Link
    _frontend -> setCamera(_camera);
    _frontend -> setBackend(_backend);
    _frontend -> setMap(_map);
    _frontend -> setDashboard(_dashboard);
    _backend -> setCamera(_camera);
    _backend -> setMap(_map);
    _dashboard -> setMap(_map);

    // Return
    return true;
}

void MonoVisualOdometry::execute() {
    cv::namedWindow("Camera Test", cv::WINDOW_AUTOSIZE);

    while (1) {
        cv::Mat _capture_mat;
        if (!_camera -> capture(_capture_mat)) break;

        cv::imshow("Camera Test", _capture_mat);
        cv::waitKey(1);

        
    }
    stop();
}

void MonoVisualOdometry::stop() {
    _camera -> close();
}

TRIAL_SLAM_NAMESPACE_END
