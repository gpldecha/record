#ifndef REPLAY_H_
#define REPLAY_H_

#include <record/record.h>
#include <future>

namespace rec{

class Replay{

    typedef enum{REPLAY,REPLAY_F,STOP,CONTINUOUS} eservice;

public:

    Replay(ros::NodeHandle& node,std::size_t hz, Service_record* ptr_record, const std::string& record_folder_);

    void publish();

    void start();

private:

   bool service_callback(record::String_cmd::Request& req, record::String_cmd::Request& resp);

   void init_marker();

   void update();

   void update_marker();

   bool load();


private:
    ros::NodeHandle *ptr_node;
    std::size_t Hz;

    // serice
    ros::ServiceServer service;

    Service_record* ptr_record;

    std::atomic<bool> b_replay;
    std::future<void> future;

    std::size_t index;

    std::string record_folder;

    // visualization marker for replay of trajectories
    ros::Publisher              marker_publisher;
    visualization_msgs::Marker  m_mid,m_start,m_end,m_list;
    int vize_type;

    // data container
    std::vector<pos_data> data;
    pos_data data_t;

    std::map<std::string,eservice> services;
};

}

#endif
