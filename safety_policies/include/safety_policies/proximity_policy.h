#ifndef PROXIMITY_POLICY_H
#define PROXIMITY_POLICY_H

#include <ros/ros.h>
#include <string>
//message_filters maybe useful if syncronization is needed
/*
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/time_sequencer.h>
*/
//#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/AccelStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>

#include <safety_policies/safe_actions/publisher_safe_action.h>
#include <safety_policies/safe_actions/dynamic_reconfigure_safe_action.h>
#include <safety_core/safe_action.h>
#include <safety_core/safe_policy.h>

#include <boost/thread/recursive_mutex.hpp>

#include <dynamic_reconfigure/server.h>
#include <safety_policies/ProximityPolicyConfig.h>

#include <yaml-cpp/yaml.h>
#include <ros/package.h>

namespace safety_policies
{

  class ProximityPolicy : public safety_core::SafePolicy
  {
    public:

      ProximityPolicy();
      ~ProximityPolicy();

      void instantiateServices(ros::NodeHandle nh);
      bool checkPolicy();
      void suggestAction();
      void publishTopics();

      void createAllRings();
      void createRingMarker(visualization_msgs::Marker& marker, int level);
      void poses_CB(const geometry_msgs::PoseArray::ConstPtr& poses);
      int getRing(float x, float y);
      void dyn_reconfigureCB(safety_policies::ProximityPolicyConfig &config, uint32_t level);
      void timer_cb(const ros::TimerEvent& event);

    private:
      bool is_obstacle_detected_;
      ros::Timer timer_;
      ros::Publisher marker_pub_;
      std::vector <ros::Subscriber> monitor_sub_;
      visualization_msgs::MarkerArray marker_array_;

      ros::Time last_detection_time_;
      double region_radius_;
      double enabling_after_timeout_;
      int regions_number_;
      int fault_region_id_;
      SafeAction* action_executer_;
      boost::recursive_mutex mutex;
      dynamic_reconfigure::Server<safety_policies::ProximityPolicyConfig> dyn_server_;
      dynamic_reconfigure::Server<safety_policies::ProximityPolicyConfig>::CallbackType dyn_server_cb_;
  };

};

#endif
