#include "trialSlam/frame.h"
#include "trialSlam/feature.h"

TRIAL_SLAM_NAMESPACE_BEGIN

unsigned long Frame::__id = 0, Frame::__keyframe_id = 0;

bool Frame::markAsKeyFrame() {
    if (_is_keyframe) {
        return false;
    }
    _keyframe_id = __keyframe_id ++;
    _is_keyframe = true;
    return true;
}

Frame::Ptr Frame::Create() {
    Frame::Ptr _frame(new Frame());
    _frame -> _id = __id ++;
    return _frame;
}

TRIAL_SLAM_NAMESPACE_END
