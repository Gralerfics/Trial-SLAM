#include <gflags/gflags.h>

#include "trialSlam/vo.h"

DEFINE_int32(cameraIndex, 0, "Define the camera index when vpath is empty");

DEFINE_bool(cameraTest, true, "Show camera test window");

DEFINE_string(configPath, "./config/default.yaml", "Config file path");

int main(int argc, char **argv) {
    // Arguments
    google::ParseCommandLineFlags(&argc, &argv, true);

    // MonoCamera Configuration
    std::shared_ptr<trialSlam::MonoCamera> monoCamera(new trialSlam::MonoCamera(
        816.011676, 811.145514, 349.590470, 230.759915,
        0.040870, 0.181320, -0.003507, 0.006012, 0.000000
    ));
    monoCamera -> setCameraIndex(FLAGS_cameraIndex);
    
    // Visual Odometry
    std::shared_ptr<trialSlam::MonoVisualOdometry> vo(new trialSlam::MonoVisualOdometry());
    vo -> setCamera(monoCamera);
    
    // Run
    bool initialize_success = vo -> initialize();
    if (!initialize_success) {
        std::cout << "Failed to initialize MonoVO." << std::endl;
        return 1;
    }
    vo -> execute();
    
    return 0;
}
