#include "trialSlam/camera.h"

TRIAL_SLAM_NAMESPACE_BEGIN

std::string Camera::_toString() const {
    std::ostringstream os;
    os << "Camera[ ";
    os << "fx = " << _fx << ", ";
    os << "fy = " << _fy << ", ";
    os << "cx = " << _cx << ", ";
    os << "cy = " << _cy << ", ";
    os << "k1 = " << _k1 << ", ";
    os << "k2 = " << _k2 << ", ";
    os << "k3 = " << _k3 << ", ";
    os << "p1 = " << _p1 << ", ";
    os << "p2 = " << _p2 << " ]";
    return os.str();
}

Vec3 Camera::world2camera(const Vec3& p_w, const SE3& pose) const {
    // 此处 pose 是相机如何从原点移动到当前位置, 也即 T_wc.
    return pose.inverse() * p_w;
}

Vec3 Camera::camera2world(const Vec3& p_c, const SE3& pose) const {
    return pose * p_c;
}

Vec2 Camera::camera2pixel(const Vec3& p_c) const {
    return Vec2(
        _fx * p_c(0, 0) / p_c(2, 0) + _cx,
        _fy * p_c(1, 0) / p_c(2, 0) + _cy
    );
}

Vec3 Camera::pixel2camera(const Vec2& p_uv, double depth) const {
    return Vec3(
        (p_uv(0, 0) - _cx) / _fx * depth,
        (p_uv(1, 0) - _cy) / _fy * depth,
        depth
    );
}

Vec2 Camera::world2pixel(const Vec3& p_w, const SE3& pose) const {
    return camera2pixel(world2camera(p_w, pose));
}

Vec3 Camera::pixel2world(const Vec2& p_uv, const SE3& pose, double depth) const {
    return camera2world(pixel2camera(p_uv, depth), pose);
}

std::string MonoCamera::_toString() const {
    std::ostringstream os;
    os << "Mono" << Camera::_toString();
    return os.str();
}

std::ostream& operator <<(std::ostream& output, const Camera& camera) {
    return output << camera._toString();
}

TRIAL_SLAM_NAMESPACE_END
