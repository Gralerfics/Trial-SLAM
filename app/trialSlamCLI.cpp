#include <gflags/gflags.h>

#include "trialSlam/visual_odometry.h"
#include "trialSlam/config.h"

DEFINE_string(configPath, "../config/default.yaml", "Config file path");

int main(int argc, char **argv) {
    // Arguments
    google::ParseCommandLineFlags(&argc, &argv, true);

    // Configurations
    if (!trialSlam::Config::setPath(FLAGS_configPath)) return false;
    
    // Camera
    std::shared_ptr<trialSlam::Camera> camera(new trialSlam::Camera(
        trialSlam::Config::get<double>("cam0.fx"),
        trialSlam::Config::get<double>("cam0.fy"),
        trialSlam::Config::get<double>("cam0.cx"),
        trialSlam::Config::get<double>("cam0.cy"),
        trialSlam::Config::get<double>("cam0.k1"),
        trialSlam::Config::get<double>("cam0.k2"),
        trialSlam::Config::get<double>("cam0.k3"),
        trialSlam::Config::get<double>("cam0.p1"),
        trialSlam::Config::get<double>("cam0.p2"),
        trialSlam::Config::get<double>("cam0.index"),
        trialSlam::Config::get<double>("cam0.width"),
        trialSlam::Config::get<double>("cam0.height")
    ));

    // Map
    std::shared_ptr<trialSlam::Map> map(new trialSlam::Map());
    map -> _num_active_keyframes = trialSlam::Config::get<int>("framenum.active");

    // Dashboard
    std::shared_ptr<trialSlam::Dashboard> dashboard(new trialSlam::Dashboard());
    dashboard -> setCamera(camera);
    dashboard -> setMap(map);

    // Frontend
    std::shared_ptr<trialSlam::Frontend> frontend(new trialSlam::Frontend());
    frontend -> setCamera(camera);
    frontend -> setDashboard(dashboard);
    frontend -> setMap(map);
    frontend -> _fd_quality_level = trialSlam::Config::get<double>("detector.qualitylevel");
    frontend -> _fd_min_distance = trialSlam::Config::get<double>("detector.mindistance");
    frontend -> _num_features = trialSlam::Config::get<int>("featnum");
    frontend -> _num_features_for_init = trialSlam::Config::get<int>("featnum.init");
    frontend -> _num_features_for_keyframe = trialSlam::Config::get<int>("featnum.keyframe");

    // VisualOdometry
    std::shared_ptr<trialSlam::VisualOdometry> vo(new trialSlam::VisualOdometry());
    vo -> setFrontend(frontend);

    // Main
    if (!vo -> initialize()) return false;
    vo -> execute();

    return 0;
}
