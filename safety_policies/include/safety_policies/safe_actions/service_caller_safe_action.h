#ifndef SERVICE_CALLER_SAFE_ACTION_H
#define SERVICE_CALLER_SAFE_ACTION_H

#include <safety_core/safe_action.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>

namespace safety_policies{
    class ServiceCallerSafeAction : public safety_core::SafeAction{
        public:
            ServiceCallerSafeAction();
            ~ServiceCallerSafeAction();
            virtual void execute();
            virtual void stop();
    };
};

#endif
