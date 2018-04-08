#include <ros/ros.h>
#include <iostream>
#include "highlevel_controller/base_parameter.h"
#include "highlevel_controller/condition.h"
#include "highlevel_controller/stateMachine.h"

Parameters* param_ptr;

int main(int argc, char *argv[]){

    ros::init(argc, argv, "highlevel_controller");

    //do not relocate this code. it must locate under the declaration of nodeHandle
    param_ptr = Parameters::getInstance();

    //get ha frequency
    int ha_frequency = param_ptr->frequency;

    //init hybridautomata
    HybridAutomata ha;
    {
        //init
        ha.setState(INIT, init);
        ha.setState(TOWARD_GOAL, toward_goal);
        ha.setCondition(INIT, new Init_to_toward_goal(), TOWARD_GOAL);
        ha.setState(DONE, done);
        ha.setCondition(TOWARD_GOAL, new Toward_goal_to_done(), DONE);

        //register crosswalk
        if (param_ptr->use_process_crosswalk){
            ha.setState(PROCESS_CROSSWALK, process_crosswalk);
            
            int crosswalk_check_frequency = ha_frequency * param_ptr->crosswalk_check_duration; 
            ha.setCondition(
                TOWARD_GOAL,
                new Toward_goal_to_process_crosswalk(crosswalk_check_frequency),
                PROCESS_CROSSWALK
            );
            ha.setCondition(
                PROCESS_CROSSWALK,
                new Process_crosswalk_to_toward_goal(),
                TOWARD_GOAL
            );
        }

        //register movingobj
        if (param_ptr->use_process_movingobj){
            ha.setState(PROCESS_MOVINGOBJ, process_movingobj);

            int movingobj_check_frequency = ha_frequency * param_ptr->movingobj_check_duration;
            ha.setCondition(
                TOWARD_GOAL,
                new Toward_goal_to_process_movingobj(movingobj_check_frequency),
                PROCESS_MOVINGOBJ
            );
            ha.setCondition(
                PROCESS_MOVINGOBJ,
                new Process_movingobj_to_toward_goal(),
                TOWARD_GOAL
            );
        }

        //register parking
        if (param_ptr->use_process_parking){
            ha.setState(PROCESS_PARKING, process_parking);

            int parking_check_frequency = ha_frequency * param_ptr->parking_check_duration;
            ha.setCondition(
                TOWARD_GOAL,
                new Toward_goal_to_process_parking(parking_check_frequency),
                PROCESS_PARKING
            );
            ha.setCondition(
                PROCESS_PARKING,
                new Process_parking_to_toward_goal(),
                TOWARD_GOAL
            );
        }

        //register recovery
        if (param_ptr->use_process_recovery){
            ha.setState(PROCESS_RECOVERY, process_recovery);

            int recovery_check_frequency = ha_frequency * param_ptr->recovery_check_duration;
            ha.setCondition(
                TOWARD_GOAL,
                new Toward_goal_to_process_recovery(recovery_check_frequency),
                PROCESS_RECOVERY
            );
            ha.setCondition(
                PROCESS_RECOVERY,
                new Process_recovery_to_toward_goal(),
                TOWARD_GOAL
            );
        }
    }

    ros::Rate loop_rate(param_ptr->frequency);
    while(ros::ok()){
        param_ptr->load_param();
        ha.operate();
        loop_rate.sleep();    
    }
}