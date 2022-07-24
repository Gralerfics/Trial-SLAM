#include <gflags/gflags.h>

#include "trialSlam/visual_odometry.h"
#include "trialSlam/config.h"

DEFINE_string(configPath, "../config/default.yaml", "Config file path");

int main(int argc, char **argv) {
    // Arguments
    google::ParseCommandLineFlags(&argc, &argv, true);

    // Configurations
    if (!trialSlam::Config::setPath(FLAGS_configPath)) return false;
    
    
    
    return 0;
}
