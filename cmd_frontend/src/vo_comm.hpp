#pragma once
#include "cmd_comm.hpp"
#include "communicator.hpp"
using namespace cmd;

class VoComm : private FrontEndComm {
 public:
  VoComm(std::string ip, std::string port);
  virtual ~VoComm() {}
};