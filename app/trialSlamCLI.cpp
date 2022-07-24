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
        trialSlam::Config::get<double>("cam0.p2")
    ));
    camera -> setCameraIndex(0);

    // Map
    std::shared_ptr<trialSlam::Map> map(new trialSlam::Map());

    // Frontend
    std::shared_ptr<trialSlam::Frontend> frontend(new trialSlam::Frontend());
    frontend -> setCamera(camera);
    frontend -> setMap(map);

    // VisualOdometry
    std::shared_ptr<trialSlam::VisualOdometry> vo(new trialSlam::VisualOdometry());
    vo -> setFrontend(frontend);

    // Main
    if (!vo -> initialize()) return false;
    vo -> execute();

    return 0;
}
