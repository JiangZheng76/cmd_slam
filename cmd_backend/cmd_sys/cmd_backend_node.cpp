#include "backend.hpp"
#include <ros/ros.h>
#include <typedefs_backend.hpp>

using namespace cmd;



int main(int argc, char* argv[]){
    static LoggerPtr g_logger_backend = SYLAR_LOG_NAME("Backend");
    SYLAR_LOG_INFO(g_logger_backend) << "+++ START cmd_slam +++";
    ros::init(argc, argv, "CODSV_BackEnd");
    // ros::NodeHandle nh("~");
    CmdBackendPtr backend(new CmdBackend());
    ros::spin();
    SYLAR_LOG_INFO(g_logger_backend) << "--- END cmd_slam ---";
    return 0;
    
}