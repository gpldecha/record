#include "optitrack_rviz/input.h"
#include "record/record_forcetorque.h"
#include <boost/lexical_cast.hpp>

void callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    ROS_INFO("in callback");

}

int main(int argc, char** argv){

    /// input parameters

    std::map<std::string,std::string> input;

    input["-topic_name"]                = "";
    input["-save"]                      = "/home/guillaume/data";
    input["-rate"]                      = "100";

    opti_rviz::Input::process_input(argc,argv,input);
    opti_rviz::Input::print_input_options(input);

    ros::init(argc, argv,"record_ft_node");
    ros::NodeHandle                     node;
    rec::Record_force_torque            record_force_torque(node,input["-save"] + "force_torque",input["-topic_name"]);
    std::vector<rec::RSubscriber*>      subscribers = {&record_force_torque};
    rec::Service_record                 service_record(node,subscribers);


    ros::spin();


    return 0;
}
