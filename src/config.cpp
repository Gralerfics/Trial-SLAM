#include "trialSlam/config.h"

TRIAL_SLAM_NAMESPACE_BEGIN

bool Config::setPath(const std::string& file_name) {
    if (!_config) _config = std::shared_ptr<Config>(new Config());
    
    _config -> _file = cv::FileStorage(file_name.c_str(), cv::FileStorage::READ);

    if (!_config -> _file.isOpened()) {
        LOG(ERROR) << file_name << " not found.";
        _config -> _file.release();
        return false;
    }

    return true;
}

std::shared_ptr<Config> Config::_config = nullptr;

TRIAL_SLAM_NAMESPACE_END
