#include <record/replay.h>
#include <boost/filesystem.hpp>

namespace rec{

Replay::Replay(ros::NodeHandle &node, std::size_t hz, Record *record, const std::string &record_folder_){

    ptr_node = &node;
    // publisher
    marker_publisher = node.advertise<visualization_msgs::Marker>("visualization_marker_replay", 10);
    //service
    service = node.advertiseService("replay",&Replay::service_callback,this);
    services["replay"]      = REPLAY;
    services["freplay"]     = REPLAY_F;
    services["stop"]        = STOP;
    services["continous"]   = CONTINUOUS;

    Hz = hz;

    ptr_record = record;
    record_folder = record_folder_;
    b_replay=false;
}


void Replay::init_marker(){
    m_mid.points.resize(1);
    m_mid.type   = visualization_msgs::Marker::SPHERE_LIST;
    //  m_mid.action = visualization_msgs::Marker::DELETE;
    m_mid.header.frame_id = ptr_record->get_fixed_frame_name();
    m_mid.header.stamp = ros::Time();
    m_mid.id = 1;
    m_mid.scale.x = 0.07;
    m_mid.scale.y = 0.07;
    m_mid.scale.z = 0.07;
    m_mid.color.r = 1.0f;
    m_mid.color.g = 1.0f;
    m_mid.color.b = 1.0f;
    m_mid.color.a = 1.0f;

    m_start.points.resize(1);
    m_start.type   = visualization_msgs::Marker::SPHERE_LIST;
   // m_start.action = visualization_msgs::Marker::DELETE;
    m_start.header.frame_id = ptr_record->get_fixed_frame_name();
    m_start.header.stamp = ros::Time();
    m_start.id = 2;
    m_start.scale.x = 0.07;
    m_start.scale.y = 0.07;
    m_start.scale.z = 0.07;
    m_start.color.r = 1.0f;
    m_start.color.a = 1.0f;
    m_start.points[0].x = data[0].x;
    m_start.points[0].y = data[0].y;
    m_start.points[0].z = data[0].z;

    m_end.points.resize(1);
    m_end.type   = visualization_msgs::Marker::SPHERE_LIST;
   // m_end.action = visualization_msgs::Marker::DELETE;
    m_end.header.frame_id = ptr_record->get_fixed_frame_name();
    m_end.header.stamp = ros::Time();
    m_end.id = 3;
    m_end.scale.x = 0.07;
    m_end.scale.y = 0.07;
    m_end.scale.z = 0.07;
    m_end.color.g = 1.0f;
    m_end.color.a = 1.0f;
    m_end.points[0].x = data[data.size()-1].x;
    m_end.points[0].y = data[data.size()-1].y;
    m_end.points[0].z = data[data.size()-1].z;



    m_list.points.resize(data.size());
    m_list.type = visualization_msgs::Marker::LINE_STRIP;
    m_list.header.stamp = ros::Time();
    m_list.header.frame_id = ptr_record->get_fixed_frame_name();
    m_list.id = 4;
    m_list.scale.x = 0.02;
    m_list.color.g = 0.7;
    m_list.color.a = 0.8;
    for(std::size_t i = 0; i < data.size();i++){
        m_list.points[i].x = data[i].x;
        m_list.points[i].y = data[i].y;
        m_list.points[i].z = data[i].z;
    }

    m_mid.action   = visualization_msgs::Marker::ADD;
    m_list.action  = visualization_msgs::Marker::ADD;
    m_start.action = visualization_msgs::Marker::ADD;
    m_end.action   = visualization_msgs::Marker::ADD;



    ROS_INFO("init hand markers done!");
}

void Replay::update_marker(){


    std::cout<< "(" << data[index].x << "," << data[index].y << "," << data[index].z << ")" << std::endl;


}

void Replay::start(){
    load();

    index=0;
    b_replay=true;
    marker_publisher = ptr_node->advertise<visualization_msgs::Marker>("visualization_marker_replay", 10);
    init_marker();

    future = std::async(std::launch::async, std::bind(&Replay::update, this));


}


void Replay::update(){

         ros::Rate rate(Hz);
         index=0;
         while(ptr_node->ok() && b_replay){


             m_mid.header.stamp   = ros::Time();
             m_end.header.stamp   = ros::Time();
             m_start.header.stamp = ros::Time();
             m_list.header.stamp  = ros::Time();

             m_mid.points[0].x =  data[index].x;
             m_mid.points[0].y =  data[index].y;
             m_mid.points[0].z =  data[index].z;
             index++;

             if(index >= data.size()){
                 m_mid.action   = visualization_msgs::Marker::DELETE;
                 m_list.action  = visualization_msgs::Marker::DELETE;
                 m_start.action = visualization_msgs::Marker::DELETE;
                 m_end.action   = visualization_msgs::Marker::DELETE;
                 sleep(2);
                 b_replay=false;
                 ROS_INFO("replay finshed!");
             }

             marker_publisher.publish(m_mid);
             marker_publisher.publish(m_end);
             marker_publisher.publish(m_list);
             marker_publisher.publish(m_start);


             ros::spinOnce();
             rate.sleep();
         }


}


bool Replay::service_callback(record::String_cmd::Request& req, record::String_cmd::Request& resp){

    eservice request;

    try {
        ROS_INFO("req.str: %s",req.str.c_str());
        request = services.at(req.str);
    }
    catch (const std::out_of_range& e) {
        ROS_ERROR("no such service [%s]",req.str.c_str());
        resp.str = "service failed, no such service: " + req.str;
        return false;
    }

    switch (request) {
    case REPLAY_F:
    {
        start();
        break;
    }
    case REPLAY:
    {
        b_replay=true;
        break;
    }
    case STOP:
    {
        break;
    }
    case CONTINUOUS:
    {
        break;
    }
    }


    return true;
}

bool Replay::load(){
    data.clear();
    std::string file_to_load;

    if(ptr_record->get_last_saved_file() == ""){
        ROS_INFO("record_folder: %s",record_folder.c_str());
        boost::filesystem::path save_dir(record_folder);
        boost::filesystem::directory_iterator end_iter;

        if (boost::filesystem::exists(save_dir) && boost::filesystem::is_directory(save_dir)){
            for( boost::filesystem::directory_iterator dir_iter(save_dir) ; dir_iter != end_iter ; ++dir_iter){
                if (boost::filesystem::is_regular_file(dir_iter->status()) ){
                    file_to_load = (*dir_iter).path().string();
                    break;
                }
            }
        }
    }else{
        file_to_load = ptr_record->get_last_saved_file();
    }


    std::ifstream file(file_to_load, std::ios_base::in);
    if(file.is_open()){

        ROS_INFO("loading: %s",boost::filesystem::path(file_to_load).stem().string().c_str());

        while (!file.eof()){
            file >> data_t.x >> data_t.y >> data_t.z >> data_t.roll >> data_t.pitch >> data_t.yaw;
            data.push_back(data_t);
        }
        file.close();
       // ROS_INFO("number samples ===> %d",data.size());
        return true;
    }else{
        ROS_ERROR("failed to open: %s",file_to_load.c_str());
        return false;
    }

}

}

