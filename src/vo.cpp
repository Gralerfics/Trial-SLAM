#include "trialSlam/vo.h"

TRIAL_SLAM_NAMESPACE_BEGIN

std::shared_ptr<Camera> VisualOdometry::getCameraPtr() const {
    return _camera;
}

Camera& VisualOdometry::getCamera() const {
    return *_camera.get();
}

int VisualOdometry::getCameraType() const {
    return _camera_type;
}

void VisualOdometry::setCamera(std::shared_ptr<Camera> camera, int camera_type) {
    _camera = camera;
    _camera_type = camera_type;
}

bool VisualOdometry::initialize() {


    return true;
}

void VisualOdometry::execute() {
    
}

TRIAL_SLAM_NAMESPACE_END
