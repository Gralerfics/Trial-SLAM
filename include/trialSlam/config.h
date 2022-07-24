#pragma once
#ifndef _TRIALSLAM_CONFIG_H_
#define _TRIALSLAM_CONFIG_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Config {
    public:
        Config() {}

        ~Config() { if (_file.isOpened()) _file.release(); }

        static bool setPath(const std::string& file_name);

        template <typename T>
        static T get(const std::string& key) {
            return T(_config -> _file[key]);
        }

    private:
        cv::FileStorage _file;

        static std::shared_ptr<Config> _config;
};

TRIAL_SLAM_NAMESPACE_END

#endif
