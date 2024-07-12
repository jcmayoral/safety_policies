#include "safety_policies/proximity_policy.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

/*PLUGINLIB_DECLARE_CLASS(safety_policies, ProximityPolicy,
                        safety_policies::ProximityPolicy,
                        safety_core::SafePolicy)
*/
PLUGINLIB_EXPORT_CLASS(safety_policies::ProximityPolicy,
                        safety_core::SafePolicy)
using namespace safety_core;
namespace safety_policies
{

  void ProximityPolicy::instantiateServices(ros::NodeHandle nh){
    std::string path = ros::package::getPath("safety_policies");
    std::string config_path;
    std::string config_file;
    nh.param<std::string>("config_path", config_path, "config");
    nh.param<std::string>("config_file", config_file, "proximity_monitor.yaml");
    std::cout << path+"/"+config_path+"/"+config_file << std::endl;
    YAML::Node config_yaml = YAML::LoadFile((path+"/"+config_path+"/"+config_file).c_str());
    //std::ifstream fin((path+"/"+config_path+"/"+config_file).c_str());
    //YAML::Parser parser(fin);

    //YAML::Node doc;
    //while(parser.GetNextDocument(doc)) {
    const YAML::Node& topics_to_subscribe = config_yaml["topics"];

    std::cout << "topics " << topics_to_subscribe.size() << std::endl;

    //for (YAML::Iterator a= topics_to_subscribe.begin(); a != topics_to_subscribe.end(); ++a){
    for (YAML::const_iterator a= topics_to_subscribe.begin(); a != topics_to_subscribe.end(); ++a){
      std::string topic = a->as<std::string>();
      //std::string topic;
      //*a >> topic;
      ROS_INFO_STREAM("Subscribing to " << topic);
      monitor_sub_.push_back(nh.subscribe(topic.c_str(), 1, &ProximityPolicy::poses_CB, this));
    //}
    }

    marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("proximity_visualization", 1);
  }

  void ProximityPolicy::poses_CB(const geometry_msgs::PoseArray::ConstPtr& poses){
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    bool warning_zone = false;
    last_detection_time_ = ros::Time::now();

    for (int i = 0; i < poses->poses.size(); i++) {
      if (getRing(poses->poses[i].position.x, poses->poses[i].position.y) == 0){//FIRST POINT IN DANGER ZONE.. Return
        fault_region_id_ = 0;
        last_detection_time_ = ros::Time::now();
        return;
      }

      if (getRing(poses->poses[i].position.x, poses->poses[i].position.y) == 1){
        fault_region_id_ = 1;
        warning_zone = true;
        is_obstacle_detected_ = true;
      }

      /*
      if (getRing(rgb_cloud.points[i].x, rgb_cloud.points[i].y) > 1){
        rgb_cloud.points[i].g = 255;
      }
      */

    }

    if (!warning_zone){
        fault_region_id_ = 2;
        is_obstacle_detected_ = false;
    }

  }

  int ProximityPolicy::getRing(float x, float y){
    float distance = 0.0;
    distance = sqrt(pow(x,2) + pow(y,2));
    return int(distance/region_radius_);
  }

  ProximityPolicy::ProximityPolicy(): is_obstacle_detected_(false), region_radius_(2.5),
                                        regions_number_(3), action_executer_(NULL),
                                        fault_region_id_(0), enabling_after_timeout_(5.0)
  {
    ros::NodeHandle nh;
    timer_ = nh.createTimer(ros::Duration(1), &ProximityPolicy::timer_cb, this);
    last_detection_time_ = ros::Time::now();
    policy_.id_ = "PROXIMITY_POLICY";
    policy_.action_ =  -1;
    policy_.state_ = PolicyDescription::UNKNOWN;
    policy_.type_ = PolicyDescription::ACTIVE;

    dyn_server_cb_ = boost::bind(&ProximityPolicy::dyn_reconfigureCB, this, _1, _2);
    dyn_server_.setCallback(dyn_server_cb_);
    ROS_INFO("Constructor ProximityPolicy");
  }

  void ProximityPolicy::timer_cb(const ros::TimerEvent& event){
    ROS_INFO_STREAM("TIMER CB" << ros::Time::now().toSec() - last_detection_time_.toSec());
    ROS_INFO_STREAM("TIMEOUT " << enabling_after_timeout_);

    createAllRings();
    publishTopics();
    if (ros::Time::now().toSec() - last_detection_time_.toSec() > enabling_after_timeout_){
      ROS_ERROR("Enable default navigation... PAY ATTENTION");
      if (action_executer_!= NULL){
        action_executer_->stop();
        action_executer_ = NULL;
      }
      policy_.action_ = -1;
      policy_.state_ = PolicyDescription::SAFE;
      last_detection_time_ = ros::Time::now();
      fault_region_id_ = 2;
    }

  }

  ProximityPolicy::~ProximityPolicy()
  {
    delete action_executer_;
  }

  void ProximityPolicy::dyn_reconfigureCB(safety_policies::ProximityPolicyConfig &config, uint32_t level){
    region_radius_ = 0.5;//config.region_radius;
    enabling_after_timeout_ = config.sensor_timeout;
    createAllRings();
  }

  void ProximityPolicy::createAllRings(){
    visualization_msgs::Marker marker;
    marker_array_.markers.clear();
    for (auto i =1 ; i<=regions_number_; i++){
      createRingMarker(marker, i);
      marker_array_.markers.push_back(marker);
    }
  }

  void ProximityPolicy::createRingMarker(visualization_msgs::Marker& marker, int level){
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "proximity_regions"+std::to_string(level);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration(1.0);
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;//region_radius_ * level;
    //marker.scale.y = region_radius_ * level;
    //marker.scale.z = 0.05;
    switch(fault_region_id_){
      case 0: 
        ROS_ERROR("red");
        marker.color.r = 1.0f;// /level;
        break;
      case 1:
        ROS_WARN("Yello");
        marker.color.r = 1.0f;// /level;
        marker.color.g = 1.0f;// /level;
        break;
      case 2:
        ROS_WARN("GREEN");
        marker.color.g = 1.0f;// /level;
        break;
    }
    marker.color.a = 1.0;

    geometry_msgs::Point circumference;
    float r_2 = pow(region_radius_*level,2);

    for (float y= -region_radius_*level; y<=region_radius_*level; y+=0.1 ){
          if (pow(y,2) > r_2)
            continue;
          circumference.x = sqrt(r_2 - pow(y,2));
          circumference.y = y;
          marker.points.push_back(circumference);
    }


    //For some value 0 is never reached
    circumference.y = region_radius_*level;
    circumference.x = 0.0;
    marker.points.push_back(circumference);

    //mirror
    for (float y= region_radius_*level; y>=-region_radius_*level; y-= 0.1 ){
          if (pow(y,2) > r_2)
            continue;
          circumference.x = -sqrt(r_2 - pow(y,2));
          circumference.y = y;
          marker.points.push_back(circumference);
    }

    //For some value 0 is never reached
    circumference.y = -region_radius_*level;
    circumference.x = 0.0;
    marker.points.push_back(circumference);

  }

  void ProximityPolicy::publishTopics()
  {
    for (auto i=0; i< marker_array_.markers.size(); i++){
      marker_array_.markers[i].header.stamp = ros::Time::now();
    }

    marker_pub_.publish(marker_array_);
  }

  bool ProximityPolicy::checkPolicy()
  {
    boost::recursive_mutex::scoped_lock scoped_lock(mutex);
    //The next condition is true when is obstacle not detected and
    // and executer is initialized (obstacle not anymore on danger region)
    if(action_executer_!= NULL && !is_obstacle_detected_){
      //ROS_WARN("Stopping executer due no obstacle detected");
      //action_executer_->stop();
      //action_executer_ = NULL;
      policy_.action_ = -1;
      policy_.state_ = PolicyDescription::SAFE;
      createAllRings();
    }
    return is_obstacle_detected_;
  }

  void ProximityPolicy::suggestAction(){
    //action_executer_ = new PublisherSafeAction();
    //ROS_INFO_STREAM_THROTTLE(5, "Obstacle detected on region with ID "<< fault_region_id_ );
    switch (fault_region_id_){
      case 0: // if DANGER REGION... stop whatever is it and
      //if executer not initialized initialized it just if lock was not locked before
        if (action_executer_ != NULL){
          if (action_executer_->getSafetyID()!=0){ //overwrite other actions if initialize ignore
            action_executer_->stop();
            action_executer_ = new PublisherSafeAction();
            action_executer_->execute();
          }
        }
        else{//initialize if not
          action_executer_ = new PublisherSafeAction();
          action_executer_->execute();
        }
        break;
      case 1:
        if (action_executer_ != NULL){
          if (action_executer_->getSafetyID()==0){
            ROS_DEBUG("Obstacle previously detected on hazard zone ... ignoring Warning Zone");
            return;
          }
          if(action_executer_->getSafetyID()!=1){
            //Stop any action which is not DANGER AND WARNING
            ROS_INFO_STREAM("Stopping previous action " << action_executer_->getSafetyID());
            action_executer_->stop();
            action_executer_ = new DynamicReconfigureSafeAction();
            action_executer_->execute();
          }
        }
        else{//init if NULL
          ROS_INFO_STREAM("Stopping no previous action " << action_executer_->getSafetyID());
          return;
          action_executer_ = new DynamicReconfigureSafeAction();
          action_executer_->execute();
        }
        break;
      default:
        ROS_ERROR("Error detecting obstacles");
        break;
    }
    //createAllRings();
    policy_.state_ = PolicyDescription::UNSAFE;
    policy_.action_ = action_executer_->getSafetyID();
  }
}
