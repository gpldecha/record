#ifndef RECORD_FORCE_TORQUE_H_
#define RECORD_FORCE_TORQUE_H_

#include "record/record.h"

// STL

#include <string>
#include <ros/ros.h>

// ROS

#include <geometry_msgs/WrenchStamped.h>

namespace rec{

class Record_force_torque : public RSubscriber {

public:

    Record_force_torque(ros::NodeHandle& node,const std::string& file_name,const std::string& topic_name);

private:

    void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

private:

    std::vector<float>  one_data;
    ros::Subscriber     sub;


};

}

#endif
