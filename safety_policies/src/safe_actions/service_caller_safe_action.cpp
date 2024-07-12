#include <safety_policies/safe_actions/service_caller_safe_action.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(safety_policies::ServiceCallerSafeAction,safety_core::SafeAction);
using namespace safety_policies;


ServiceCallerSafeAction::ServiceCallerSafeAction(){
    ROS_INFO("ServiceCallerSafeAction Safe Action");
    safety_id_ = 2;
};

ServiceCallerSafeAction::~ServiceCallerSafeAction(){
};

void ServiceCallerSafeAction::execute(){

};

void ServiceCallerSafeAction::stop(){
    //ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
};
