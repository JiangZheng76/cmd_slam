#include "backend.hpp"
#include <ros/ros.h>
#include <typedefs_backend.hpp>

using namespace cmd;

LoggerPtr g_logger = SYLAR_LOG_NAME("CMD-SLAM");

int main(int argc, char* argv[]){
    
    SYLAR_LOG_INFO(g_logger) << "--> START CMD-SLAM";
    ros::init(argc, argv, "CODSV_BackEnd");
    // ros::NodeHandle nh("~");
    CmdBackendPtr backend(new CmdBackend());
    ros::spin();
    SYLAR_LOG_INFO(g_logger) << "<-- END CMD-SLAM";
    return 0;
    
}