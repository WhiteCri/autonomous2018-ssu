#include "highlevel_controller/condition.h"
#include "highlevel_controller/base_parameter.h"

extern Parameters* param_ptr;

bool Init_to_toward_goal::timedCheck(HybridAutomata *HA){
    return true;
}

/* crosswalk */
bool Toward_goal_to_process_crosswalk::timedCheck(HybridAutomata *HA){
    bool ret = true;
    ret &= param_ptr->crosswalk;
    if (param_ptr->use_crosswalk_onetime_flag)
        ret &= !param_ptr->crosswalk_onetime_flag;
    
    return ret; 
}

bool Process_crosswalk_to_toward_goal::timedCheck(HybridAutomata *HA){
    return true;
}

/* movingobj */
bool Toward_goal_to_process_movingobj::timedCheck(HybridAutomata* HA){
    bool ret = true;
    ret &= param_ptr->movingobj;
    if (param_ptr->use_movingobj_onetime_flag)
        ret &= !param_ptr->movingobj_onetime_flag;
    
    return ret;
}

bool Process_movingobj_to_toward_goal::timedCheck(HybridAutomata *HA){
    return true;
}

/* parking */
bool Toward_goal_to_process_parking::timedCheck(HybridAutomata* HA){
    bool ret = true;
    ret &= param_ptr->parking;
    if (param_ptr->use_parking_onetime_flag)
        ret &= !param_ptr->parking_onetime_flag;

    return ret;
}

bool Process_parking_to_toward_goal::timedCheck(HybridAutomata *HA){
    return true;
}

/* uturn */
bool Toward_goal_to_process_uturn::timedCheck(HybridAutomata* HA){
    bool ret = true;
    ret &= param_ptr->uturn;
    if (param_ptr->use_uturn_onetime_flag)
        ret &= !param_ptr->uturn_onetime_flag;
        
    return ret;
}

bool Process_uturn_to_toward_goal::timedCheck(HybridAutomata *HA){
    return true;
}

/* recovery */
bool Toward_goal_to_process_recovery::timedCheck(HybridAutomata* HA){
    bool ret = true;
    ret &= param_ptr->recovery;

    return ret;
}

bool Process_recovery_to_toward_goal::timedCheck(HybridAutomata *HA){
    return true;
}

/* Done */
bool Toward_goal_to_done::timedCheck(HybridAutomata* HA){
    bool ret = param_ptr->reached_goal;
    return ret;
}
