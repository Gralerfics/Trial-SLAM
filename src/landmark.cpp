#include "trialSlam/landmark.h"
#include "trialSlam/feature.h"

TRIAL_SLAM_NAMESPACE_BEGIN

unsigned long Landmark::__id = 0;

void Landmark::addObservedBy(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> ulock(_mutex);
    _num_observed_bys ++;
    _observed_bys.push_back(feature);
}

void Landmark::removeObservedBy(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> ulock(_mutex);
    for (auto it = _observed_bys.begin(); it != _observed_bys.end();) {
        if (*it == feature) {
            _observed_bys.erase(it);
            feature -> getLandmark().reset();
            _num_observed_bys --;
            break;
        } else {
            it ++;
        }
    }
}

Landmark::Ptr Landmark::Create() {
    Landmark::Ptr _landmark(new Landmark());
    _landmark -> _id = __id ++;
    return _landmark;
}

TRIAL_SLAM_NAMESPACE_END
