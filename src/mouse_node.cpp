
// STL

#include <map>
#include <string>

// Boost

#include <boost/lexical_cast.hpp>

// ROS

#include <ros/ros.h>
#include <ros/service_client.h>
#include <record/String_cmd.h>
#include <record/record.h>

#include <optitrack_rviz/input.h>

// mouse

#include "mouse/mouse.h"


class Manager{

public:

    Manager(ros::ServiceClient* client,ms::Mouse& mouse):ptr_client(client),mouse(mouse){
        b_mouse = false;
        b_first = true;
        b_replay = false;

        left_cb     = std::bind( &Manager::mouse_left_callback, this);
        right_cb    = std::bind( &Manager::mouse_right_callback, this);
        middle_cb   = std::bind( &Manager::mouse_middle_callback, this);


        mouse.register_callback(left_cb,ms::LEFT);
        mouse.register_callback(right_cb,ms::RIGHT);
        mouse.register_callback(middle_cb,ms::MIDDLE);

    }

    void mouse_left_callback(){
        if(b_mouse){
             if(b_first){
                srv.request.str = "start";
                ptr_client->call(srv);
                b_first = false;
             }else{
                 srv.request.str = "stop";
                 ptr_client->call(srv);

                 srv.request.str = "save";
                 ptr_client->call(srv);

                 b_first = true;
             }
        }
    }

    void mouse_right_callback(){

    }

    void mouse_middle_callback(){
        if(!b_mouse){
            ROS_INFO("start mouse listener");
            b_mouse = true;
        }else{
            ROS_INFO("stop mouse listener");
            b_first = true;
            b_mouse = false;
        }
    }

private:

    ros::ServiceClient* ptr_client;
    record::String_cmd srv;
    ms::Mouse& mouse;
    ms::mouse_callback left_cb,right_cb,middle_cb;
    bool b_mouse;
    bool b_first,b_replay;
    std::string cmd;


};



int main(int argc,char** argv)
{
    std::map<std::string,std::string> input;

    input["-service"]                = "";


    opti_rviz::Input::process_input(argc,argv,input);
    opti_rviz::Input::print_input_options(input);


    ros::init(argc, argv, "mouse_node");
    ros::NodeHandle  node;
    ros::ServiceClient client =  node.serviceClient<record::String_cmd>(input["-service"]);


    ms::Mouse mouse;
    Manager manager(&client,mouse);
    mouse.start();

    ros::spin();

	return 0;
}
