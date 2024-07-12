#include <safety_policies/policies_manager.h>


using namespace safety_policies;


PoliciesManager::PoliciesManager(): nh_(){

}

PoliciesManager::~PoliciesManager(){

}

void PoliciesManager::addPolicy(safety_core::SafePolicy* new_policy){
    new_policy->instantiateServices(nh_);
    policies_.push_back(new_policy);
}

void PoliciesManager::monitor(){
    for (auto i=0; i< policies_.size(); i++){
        if(policies_[i]->checkPolicy()){
            //ROS_WARN("Proximity monitor activated");
            policies_[i]->reportState();
        }

        //monitors_[i]->publishTopics();
    }

}
