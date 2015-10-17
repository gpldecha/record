#include "optitrack_rviz/input.h"
#include "record/record_position.h"
#include <boost/lexical_cast.hpp>
#include "record/record_forcetorque.h"

int main(int argc, char** argv){

    /// input parameters

    std::map<std::string,std::string> input;

    input["-topic_name"]                = "";
    input["-fixed_frame"]               = "";
    input["-target_frame_listener"]     = "";
    input["-save"]                      = "/home/guillaume/data";
    input["-rate"]                      = "100";

    opti_rviz::Input::process_input(argc,argv,input);
    opti_rviz::Input::print_input_options(input);

    ros::init(argc, argv,"~",ros::init_options::AnonymousName);
    ros::NodeHandle                     node;
    rec::Record_position                record_position(input["-save"] + "position",input["-fixed_frame"],input["-target_frame_listener"]);
    rec::Record_force_torque            record_force_torque(node,input["-save"] + "force_torque",input["-topic_name"]);
    std::vector<rec::RSubscriber*>      subscribers = {&record_force_torque,&record_position};
    rec::Service_record                 service_record(node,subscribers);


    int Hz = boost::lexical_cast<int>(input["-rate"]);
    ros::Rate rate(Hz);
    while(node.ok()){

        record_position.update();

        ros::spinOnce();
        rate.sleep();
    }



}
