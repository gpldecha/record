#ifndef RECORD_POSITION_H_
#define RECORD_POSITION_H_

#include "record/record.h"
#include <optitrack_rviz/listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

namespace rec{

class Record_position  : public RSubscriber {

public:

    Record_position(const std::string& file_name,const std::string& fixed_frame,const std::string& target_frame);

    void update();


private:



    opti_rviz::Listener listener;
    tf::Vector3         origin;
    tf::Matrix3x3       orientation;
    geometry_msgs::Pose pos_msg;
    tf::Quaternion      q;
    std::vector<float>  one_data;




};

}


#endif
