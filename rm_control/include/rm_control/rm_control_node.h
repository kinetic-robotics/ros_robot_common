#ifndef RM_CONTROL_MODULE_RM_CONTROL_NODE_H_
#define RM_CONTROL_MODULE_RM_CONTROL_NODE_H_

namespace rm_control
{
enum class ShotStatus {
    NONE                = 0,
    SHOT_ONCE           = 1,
    SHOT_CONTINOUS      = 2,
    SHOT_CONTINOUS_STOP = 3,
};

}  // namespace rm_control
#endif