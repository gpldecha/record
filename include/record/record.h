#ifndef RECORD_H_
#define RECORD_H_

// STL

#include <vector>
#include <fstream>
#include <map>

// ROS

#include <ros/service.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "record/String_cmd.h"

#include <rosbag/bag.h>

namespace rec{


class Data{

public:

    Data(const std::string &filename);

    void push_back(const std::vector<float>& data);

    bool save();

    void clear();

private:

    std::string current_date_time() const;


private:

    const std::string                   filename;
    std::vector<std::vector<float> >    all_data;
    rosbag::Bag                         bag;

};

class RSubscriber{

public:

    RSubscriber(const std::string &filename);

    void start();

    void stop();

    void reset();

    void save();

protected:

    bool            bRecord;
    ros::Subscriber subscriber;
    std::string     full_name;
    rec::Data       data;




};

class Service_record{

    typedef enum{START,STOP,RESET,SAVE} rservice;

public:

    Service_record(ros::NodeHandle& node,std::vector<RSubscriber*>& rsubscribers);


private:


    bool service_callback(record::String_cmd::Request& req, record::String_cmd::Request& resp);

private:

    std::vector<RSubscriber*>&     rsubscriber;

    ros::ServiceServer service;
    std::map<std::string,rservice> services;

};


}

#endif
