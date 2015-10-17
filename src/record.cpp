#include "record/record.h"

namespace rec{

Data::Data(const std::string &filename):
    filename(filename)
{

}

void Data::push_back(const std::vector<float>& data){
    all_data.push_back(data);
   // bag.write()
}

bool Data::save(){
    if(all_data.size() == 0){
        std::cerr<< "data.size = 0, no data to save" << std::endl;
        return false;
    }

    std::ofstream file;

    std::string file_name_ = filename + "_" + current_date_time() + ".txt";
    file.open(file_name_);
 //   bag.open(filename + "_" + current_date_time() + ".bag");

    if(file){
        std::vector<float> tmp;
        for(size_t i = 0; i < all_data.size();i++){
            tmp = all_data[i];
            file << std::setprecision(5);
            for(std::size_t j = 0; j < all_data[i].size()-1;j++){
                file << all_data[i][j]  << ",";
            }
            file << all_data[i][all_data[i].size()-1] << std::endl;
        }
        file.close();
        std::cout<< "file: " << file_name_ << "\tsaved !" << std::endl;
        std::cout<< "number of data points: " << all_data.size() << std::endl;
    }else{
        std::cout<< "failed to open : " << file_name_ << " !" << std::endl;
        return false;
    }
    return true;
}

void Data::clear(){
    all_data.clear();
}

std::string Data::current_date_time() const {
    std::time_t     now = time(0);
    std::tm *localTime = localtime(&now);
    std::string time("");

    int Day    = localTime->tm_mday;
    int Month  = localTime->tm_mon + 1;
    int Year   = localTime->tm_year + 1900;
    int Hour   = localTime->tm_hour;
    int Min    = localTime->tm_min;
    int Sec    = localTime->tm_sec;


    time = time + std::string(boost::lexical_cast<std::string>(Day))    + "-"
            + std::string(boost::lexical_cast<std::string>(Month))  + "-"
            + std::string(boost::lexical_cast<std::string>(Year))   + "_"
            + std::string(boost::lexical_cast<std::string>(Hour))   + "h"
            + std::string(boost::lexical_cast<std::string>(Min))    + "m"
            + std::string(boost::lexical_cast<std::string>(Sec))    + "s";

    return time;
}


RSubscriber::RSubscriber(const std::string& filename):data(filename){
    bRecord = false;
}

void RSubscriber::start(){
    ROS_INFO("start recording!");
    bRecord=true;
}

void RSubscriber::stop(){
    ROS_INFO("stop recording!");
    bRecord=false;
}

void RSubscriber::reset(){
    ROS_INFO("reset recording!");
    stop();
    data.clear();
    ROS_INFO("data buffer cleared!");
}

void RSubscriber::save(){
    ROS_INFO("save & reset recording!");
    ROS_INFO("saved ==> %s",full_name.c_str());
    stop();
    data.save();
    reset();
}


Service_record::Service_record(ros::NodeHandle& node, std::vector<RSubscriber *> &rsubscribers): rsubscriber(rsubscribers)
    {
        service    = node.advertiseService("record",&Service_record::service_callback,this);
        services["start"] = START;
        services["stop"]  = STOP;
        services["reset"] = RESET;
        services["save"]  = SAVE;
}



bool Service_record::service_callback(record::String_cmd::Request& req, record::String_cmd::Request& resp){

        rservice request;

        try {
            request = services.at(req.str);
        }
        catch (const std::out_of_range& e) {
            ROS_ERROR("no such service [%s]",req.str.c_str());
            resp.str = "service failed, no such service: " + req.str;
            return false;
        }

        switch (request) {
        case START:
        {
            for(std::size_t i = 0; i < rsubscriber.size();i++){
                rsubscriber[i]->start();
            }
            break;
        }
        case STOP:
        {
            for(std::size_t i = 0; i < rsubscriber.size();i++){
                rsubscriber[i]->stop();
            }
            break;
        }
        case RESET:
        {
            for(std::size_t i = 0; i < rsubscriber.size();i++){
                rsubscriber[i]->reset();
            }
            break;
        }
        case SAVE:
        {
            for(std::size_t i = 0; i < rsubscriber.size();i++){
                rsubscriber[i]->save();
            }
            break;
        }

        }

        resp.str = "service: " + req.str + " sucessfull!";

        return true;
    }


}
