#include "record/record_forcetorque.h"

namespace rec {


Record_force_torque::Record_force_torque(ros::NodeHandle &node, const std::string &file_name, const std::string &topic_name):
RSubscriber(file_name){

    ROS_INFO(" FT-RECORD topic: %s",topic_name.c_str());
    sub = node.subscribe(topic_name,100,&Record_force_torque::callback,this);
    one_data.resize(6);


}

void Record_force_torque::callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    if(bRecord){
        one_data[0] = msg->wrench.force.x;
        one_data[1] = msg->wrench.force.y;
        one_data[2] = msg->wrench.force.z;
        one_data[3] = msg->wrench.torque.x;
        one_data[4] = msg->wrench.torque.y;
        one_data[5] = msg->wrench.torque.z;
        data.push_back(one_data);
    }
}

}
