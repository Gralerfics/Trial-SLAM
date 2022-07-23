#include <gflags/gflags.h>

#include "trialSlam/vo.h"

DEFINE_int32(cameraIndex, 0, "Define the camera index when vpath is empty");
DEFINE_bool(cameraTest, false, "Show camera test window");

DEFINE_string(configPath, "./config/default.yaml", "Config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    
    std::shared_ptr<trialSlam::VisualOdometry> vo(new trialSlam::VisualOdometry());

    vo -> setCamera(
        std::shared_ptr<trialSlam::Camera>(new trialSlam::MonoCamera(
            816.011676, 811.145514, 349.590470, 230.759915,
            0.040870, 0.181320, -0.003507, 0.006012, 0.000000
        )),
        trialSlam::VO_CAMERA_TYPE_MONO
    );
    
    assert(vo -> initialize());

    vo -> execute();

    return 0;
}
