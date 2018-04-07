#include "condition.h"

extern Parameter param;

bool Init_to_toward_goal::check(HybridAutomata *HA){
    return true;
}

/* crosswalk */
bool Toward_goal_to_process:timedCheck(HybridAutomata *HA){
    bool ret = true;
    ret &= param.crosswalk;
    if (param.use_crosswalk_onetime_flag)
        ret &= !param.crosswalk_onetime_flag;
    return ret; 
}

bool Process_crosswalk_to_toward_goal::check(HybridAutomata *HA){
    return true;
}

/* movingobj */
bool Toward_goal_to_process_movingobj::timedCheck(HybridAutomata* HA){
    bool ret = true;
    ret &= param.movingobj;
    if (param.use_movingobj_onetime_flag)
        ret &= !param.movingobj_onetime_flag;
    return ret;
}

bool Process_movingobj_to_toward_goal::check(HybridAutomata *HA){
    return !param.movingobj;
}

/* parking */
bool Toward_goal_to_process_parking::timedCheck(HybridAutomata* HA){
    bool ret = true;
    ret &= param.parking;
    if (param.use_parking_onetime_flag)
        ret &= !param.parking_onetime_flag;
    return ret;
}

bool Process_parking_to_toward_goal::check(HybridAutomata *HA){
    return true;
}

/* recovery */
bool Toward_goal_to_process_recovery::timedCheck(HybridAutomata* HA){
    bool ret = true;
    ret &= param.recovery;
    return ret;
}

bool Process_recovery_to_toward_goal::check(HybridAutomata *HA){
    return true;
}

/* Done */
bool Toward_goal_to_done::check(HybridAutomata* HA){
    return param.reached_goal;
}
