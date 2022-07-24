#pragma once
#ifndef _TRIALSLAM_LANDMARK_H_
#define _TRIALSLAM_LANDMARK_H_

#include "trialSlam/common.h"

TRIAL_SLAM_NAMESPACE_BEGIN

class Feature;

class Landmark {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Landmark> Ptr;

        Landmark() {}

        Landmark(unsigned long id, const Vec3& position): _id(id), _position(position) {}

        unsigned long getId() {
            return _id;
        }

        Vec3 getPosition() {
            std::unique_lock<std::mutex> ulock(_mutex);
            return _position;
        }

        void setPosition(const Vec3& position) {
            std::unique_lock<std::mutex> ulock(_mutex);
            _position = position;
        }

        unsigned long getNumObservedBys() {
            std::unique_lock<std::mutex> ulock(_mutex);
            return _num_observed_bys;
        }

        std::list<std::shared_ptr<Feature>> getObservedBys() {
            std::unique_lock<std::mutex> ulock(_mutex);
            return _observed_bys;
        }

        void addObservedBy(std::shared_ptr<Feature> feature);

        void removeObservedBy(std::shared_ptr<Feature> feature);

        static Landmark::Ptr Create();

    private:
        unsigned long _id;
        // bool _excluded = false;
        Vec3 _position = Vec3::Zero();
        unsigned long _num_observed_bys = 0;
        std::list<std::shared_ptr<Feature>> _observed_bys;
        std::mutex _mutex;

        static unsigned long __id;
};

TRIAL_SLAM_NAMESPACE_END

#endif
