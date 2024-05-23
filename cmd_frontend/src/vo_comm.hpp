#pragma once 
#include "communicator.hpp"
#include "cmd_comm.hpp"
using namespace cmd;

class VoComm : private FrontEndComm{
public:
    VoComm(std::string ip,std::string port);
    virtual ~VoComm(){}

    
};