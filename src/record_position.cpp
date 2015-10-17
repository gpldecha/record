#include "record/record_position.h"

namespace rec{

Record_position::Record_position(const std::string &file_name,const std::string& fixed_frame, const std::string& target_frame):
RSubscriber(file_name),listener(fixed_frame,target_frame){

    one_data.resize(7);
}

void Record_position::update(){
    if(bRecord){
        listener.update(origin,orientation);
        orientation.getRotation(q);

        one_data[0] = origin.x();
        one_data[1] = origin.y();
        one_data[2] = origin.z();

        one_data[3] = q.x();
        one_data[4] = q.y();
        one_data[5] = q.z();
        one_data[6] = q.w();

        data.push_back(one_data);
    }
}

}


