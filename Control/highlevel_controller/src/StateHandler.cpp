#include <ros/ros.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "highlevel_controller/StateHandler.h"
#include "highlevel_controller/base_parameter.h"
#include "highlevel_controller/goalSender.h"

#define ABORT_FLAG TRUE
#define TX_STOP_BRAKE 75
#define SUCCEEDED_BUG_RESOLVE_TIME 0.5
static constexpr double LOAD_STOP_DURATION = 1.0;
static constexpr double ABORT_WAIT_DURATION = 3.0;
extern Parameters* param_ptr;
extern GoalSender* goalSender_ptr;

class DynamicParamSender{
public:
    DynamicParamSender(): node_name("/move_base/DWAPlannerROS/set_parameters")
    {}

    void saveDynamicParams(std::string name, double val){
        ROS_INFO("setting %s to %lf...", name.c_str(), val);
        dynamic_reconfigure::DoubleParameter double_param;
        double_param.name = name;
        double_param.value = val;
        conf.doubles.emplace_back(double_param);

        srv_req.config = conf;
        ros::service::call(node_name.c_str(), srv_req, srv_resp);

        conf.doubles.clear();
        conf.doubles.resize(0);
    }
private:
    std::string node_name;
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::Config conf;
};

inline void showGoal(double x, double y, double ori_z, double ori_w, const std::string& str){
    ROS_INFO("set new goal : %lf, %lf, %lf, %lf, %s",
        x, y, ori_z, ori_w, str.c_str());
}

bool handler_setGoal(bool newGoal=false){
    //if already all goals has been done, return false.
    if ((param_ptr->x_goal.size()==0)||(param_ptr->y_goal.size()==0)||(param_ptr->ori_z_goal.size()==0)
            ||(param_ptr->ori_w_goal.size()==0)||(param_ptr->goal_type.size()==0)) return false;

    //if new goal, pop
    if (newGoal){
        param_ptr->x_goal.pop_back();
        param_ptr->y_goal.pop_back();
        param_ptr->ori_z_goal.pop_back();
        param_ptr->ori_w_goal.pop_back();
        param_ptr->goal_type.pop_back();
    }
    //if any element lack, I assumes all goal has been done
    if ((param_ptr->x_goal.size()==0)||(param_ptr->y_goal.size()==0)||(param_ptr->ori_z_goal.size()==0)
            ||(param_ptr->ori_w_goal.size()==0)||(param_ptr->goal_type.size()==0)) return false;
    
    //if type is skip, pass the goal
    while (true){
        std::string goal_type = param_ptr->goal_type.back();
        if (goal_type != "skip") break;
        else {
            param_ptr->x_goal.pop_back();
            param_ptr->y_goal.pop_back();
            param_ptr->ori_z_goal.pop_back();
            param_ptr->ori_w_goal.pop_back();
            param_ptr->goal_type.pop_back();
        }
    }
    double x = param_ptr->x_goal.back();
    double y = param_ptr->y_goal.back();
    double ori_z = param_ptr->ori_z_goal.back();
    double ori_w = param_ptr->ori_w_goal.back();
    std::string goal_type = param_ptr->goal_type.back();

    goalSender_ptr->setGoal(x, y, ori_z, ori_w, goal_type);
    goalSender_ptr->sendGoal();
    showGoal(x, y, ori_z, ori_w, goal_type);

    return true;
}

static void setInitialPos(double x, double y, double ori_z, double ori_w){
    ros::Publisher pub =
        param_ptr->nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",100);
    geometry_msgs::PoseWithCovarianceStamped p;
    p.header.frame_id = "map";
    p.header.seq = 1;
    p.header.stamp = ros::Time::now();
    p.pose.pose.position.x = x;
    p.pose.pose.position.y = x;
    p.pose.pose.orientation.z = ori_z;
    p.pose.pose.orientation.w = ori_w;
    p.pose.covariance = boost::array<double, 36>({{
        0.25, 0, 0, 0, 0, 0,
        0, 0.25, 0, 0, 0, 0,
        0, 0, 0.25, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0.0685389,
    }});
    ROS_INFO("fucking initial pose set");
    pub.publish(p);
}

static void waitUntilReach(){
    while(true){
        auto state = goalSender_ptr->getState();
        if (state == GoalSender::GoalStates::STATE_SUCCEEDED){
            ROS_INFO("SUCCEEDED...");
            if (handler_setGoal(true) == false){
                ROS_INFO("reached all goals!");
                param_ptr->nh.setParam("hl_controller/reached_goal", true);
                return;
            }
            ros::Rate(1).sleep();
            ROS_INFO("success end!");
            break;
        }
    }
}


void init(){
    ROS_INFO("State Init...");
    param_ptr->nh.setParam("hl_controller/curState","INIT");
    //param_ptr->nh.setParam("hl_controller/use_base_filter",true);
setInitialPos(30000,30000,1,0);
    handler_setGoal();
}

void toward_goal(){
    typedef GoalSender::GoalStates GoalStates;
    param_ptr->nh.setParam("hl_controller/tx_control_static", false);
    param_ptr->nh.setParam("hl_controller/curState","TOWARD_GOAL");
    
    auto state = goalSender_ptr->getState();
    if (state == GoalStates::STATE_SUCCEEDED){
        ROS_INFO("SUCCEEDED...");
        if (handler_setGoal(true) == false){
            ROS_INFO("reached all goals!");
            param_ptr->nh.setParam("hl_controller/reached_goal", true);
            return;
        }
        ros::Rate((double)1/SUCCEEDED_BUG_RESOLVE_TIME).sleep();
        ROS_INFO("success end!");
    }
    else if (state == GoalStates::STATE_LOST){
        ROS_INFO("LOST...");
        handler_setGoal();
    }
    else if (state == GoalStates::STATE_ACTIVE){}
    else if (state == GoalStates::STATE_PENDING) {
        ROS_INFO("PENDING...");
    }
    else if (state == GoalStates::STATE_PREEMPTED){
        ROS_INFO("PREEMPTED...Are you using rviz to set goal?");
        handler_setGoal();
    }
    else if (state == GoalStates::STATE_ABORTED){
#ifdef ABORT_FLAG
        ROS_ERROR("waiting for 3 seconds...");
        ros::Rate(1/ABORT_WAIT_DURATION).sleep();
        
        ROS_ERROR("waiting for estop mode...");
        ros::Rate loop_rate(param_ptr->frequency);
        while(true){
            int isEstop = 0;
            param_ptr->nh.getParam("/isEstop", isEstop);
ROS_INFO("my estop : %d", isEstop);
            if (isEstop) break;
            loop_rate.sleep();
        }
        ROS_INFO("handling ABORT STATUS end...");
        ROS_ERROR("SEND NEW GOAL...");

        if (handler_setGoal(true) == false){
            ROS_INFO("reached all goals!");
            param_ptr->nh.setParam("hl_controller/reached_goal", true);
            return;
        }
#elif 
        ROS_ERROR("ABORTED... exit the program");
        exit(-1);
#endif
    }
    else {
        ROS_INFO("why control reaches here...");
    } 
}

void process_crosswalk(){
    ROS_INFO("crosswalk start");
    param_ptr->nh.setParam("hl_controller/curState","PROCESS_CROSSWALK");

    //wait until reaching goal
    ROS_INFO("wait for reaching goall...");
    waitUntilReach();

    //take car to stop
    ROS_INFO("stop for %lf seconds...", param_ptr->crosswalk_stop_duration);
    param_ptr->nh.setParam("hl_controller/tx_control_static", true);
    param_ptr->nh.setParam("hl_controller/tx_speed", 0);
    param_ptr->nh.setParam("hl_controller/tx_steer", 0);
    param_ptr->nh.setParam("hl_controller/tx_brake", TX_STOP_BRAKE);

    //take car to wait
    ros::Rate(1 / param_ptr->crosswalk_stop_duration).sleep();

    //unlock tx_control_static
    param_ptr->nh.setParam("hl_controller/tx_control_static",false);

    ROS_INFO("Unlock Tx static Control...");

    //check that process_crosswalk had been done.
    param_ptr->nh.setParam("hl_controller/crosswalk_onetime_flag",true);
    

    ROS_INFO("crosswalk done");    
    param_ptr->load_param();
    
    
}

void process_movingobj(){
    ROS_INFO("movingobj start");
    param_ptr->nh.setParam("hl_controller/curState","PROCESS_MOVINGOBJ");

    waitUntilReach();

    //take car to stop
    ROS_INFO("stop...");
    param_ptr->nh.setParam("hl_controller/tx_control_static", true);
    param_ptr->nh.setParam("hl_controller/tx_speed", 0);
    param_ptr->nh.setParam("hl_controller/tx_steer", 0);
    param_ptr->nh.setParam("hl_controller/tx_brake", TX_STOP_BRAKE);

    //take car to wait
    ros::Rate(1 / param_ptr->movingobj_stop_duration).sleep();

    ROS_INFO("Unlock Tx Static Control...");

    //unlock tx_control_static
    param_ptr->nh.setParam("hl_controller/tx_control_static",false);

    //check that process_movingobj had been done.
    param_ptr->nh.setParam("hl_controller/movingobj_onetime_flag",true);

    ROS_INFO("movingobj done");
    param_ptr->load_param();
}

void process_parking(){
    ROS_INFO("process parking start");

    //take car to stop
    ROS_INFO("stop...");
    param_ptr->nh.setParam("hl_controller/tx_control_static", true);
    param_ptr->nh.setParam("hl_controller/tx_speed", 0);
    param_ptr->nh.setParam("hl_controller/tx_steer", 0);
    param_ptr->nh.setParam("hl_controller/tx_brake", TX_STOP_BRAKE);

    DynamicParamSender ds;
    ds.saveDynamicParams("max_vel_x", -3.0);
    ds.saveDynamicParams("min_vel_x", -2.0);    

    //take car to wait
    ros::Rate(1 / param_ptr->parking_stop_duration).sleep();

    //set goal to backing point
    ROS_INFO("Unlock tx static control...");
    param_ptr->nh.setParam("hl_controller/tx_control_static", false);
    
    waitUntilReach();

    ds.saveDynamicParams("max_vel_x", 8.0);
    ds.saveDynamicParams("min_vel_x", 1.5);
    //param_ptr->nh.setParam("hl_controller/use_base_filter",false);
    
    ROS_INFO("to the backing point...");
    waitUntilReach();

    ROS_INFO("process parking done!");
    param_ptr->nh.setParam("hl_controller/parking_onetime_flag",true);
}

void process_uturn(){
    ROS_INFO("uturn start");
    param_ptr->nh.setParam("hl_controller/curState","PROCESS_UTURN");

    double uturn_duration = param_ptr->uturn_duration;

    ROS_INFO("change move_base params...");
    DynamicParamSender ds;
    ds.saveDynamicParams("acc_lim_x", 5.5);

    waitUntilReach();

    ROS_INFO("reload pre-move_base params...");
    ds.saveDynamicParams("acc_lim_x", 3.5);
    
    //check that process_movingobj had been done.
    param_ptr->nh.setParam("hl_controller/uturn_onetime_flag",true);

    ROS_INFO("uturn done");
}

void process_sload(){

    ROS_INFO("sload start");
    param_ptr->nh.setParam("hl_controller/curState","PROCESS_SLOAD");

    waitUntilReach();
     
    param_ptr->nh.setParam("hl_controller/sload_onetime_flag", true);
    ROS_INFO("sload done...");
}

void process_nload(){
    ROS_INFO("nlaod start");
    param_ptr->nh.setParam("hl_controller/curState","PROCESS_NLOAD");

    ROS_INFO("stop...");
    param_ptr->nh.setParam("hl_controller/tx_control_static", true);
    param_ptr->nh.setParam("hl_controller/tx_speed", 0);
    param_ptr->nh.setParam("hl_controller/tx_steer", 0);
    param_ptr->nh.setParam("hl_controller/tx_brake", TX_STOP_BRAKE);
    ros::Rate((double)1/LOAD_STOP_DURATION).sleep();
    
    ROS_INFO("unlock stop...");
    param_ptr->nh.setParam("hl_controller/tx_control_static", false);
    
    param_ptr->nh.setParam("hl_controller/nload_onetime_flag", true);
    ROS_INFO("nload done...");
}

void process_recovery(){
    param_ptr->nh.setParam("hl_controller/recovery",false);
    param_ptr->nh.setParam("hl_controller/curState","PROCESS_RECOVERY");
}

void done(){
    param_ptr->nh.setParam("hl_controller/curState","DONE");
    ROS_INFO("ALL GOAL had been processed");
}