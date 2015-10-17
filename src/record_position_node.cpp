#include "optitrack_rviz/input.h"
#include "record/record_position.h"
#include <boost/lexical_cast.hpp>

int main(int argc, char** argv){

    /// input parameters

    std::map<std::string,std::string> input;

    input["-fixed_frame"]               = "";
    input["-target_frame_listener"]     = "";
    input["-save"]                      = "/home/guillaume/";
    input["-rate"]                      = "100";

    opti_rviz::Input::process_input(argc,argv,input);
    opti_rviz::Input::print_input_options(input);

    ros::init(argc, argv,"~",ros::init_options::AnonymousName);
    ros::NodeHandle                     node;
    rec::Record_position                record_position(input["-save"] + "/position",input["-fixed_frame"],input["-target_frame_listener"]);
    std::vector<rec::RSubscriber*>      subscribers = {&record_position};
    rec::Service_record                 service_record(node,subscribers);



    int Hz = boost::lexical_cast<int>(input["-rate"]);
    ros::Rate rate(Hz);
    while(node.ok()){

        record_position.update();

        ros::spinOnce();
        rate.sleep();
    }



}
