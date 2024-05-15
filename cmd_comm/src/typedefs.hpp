#pragma once

#include <memory>
#include <vector>
#include <list>
#include <sstream>

// thrid-party

// sylar
#include <log.h>
#include <address.h>
#include <bytearray.h>
#include <socket_stream.h>
// using namespace mysylar;
namespace cmd{

using ByteArray = mysylar::ByteArray; 
using ByteArrayPtr = mysylar::ByteArray::ptr; // 封装好的字节流工具

using SocketStream = mysylar::SocketStream;
using SocketStreamPtr = mysylar::SocketStream;

using IPv4Address = mysylar::IPv4Address;
using IPv4AddressPtr = mysylar::Address::ptr;

using Socket = mysylar::Socket;
using SocketPtr = mysylar::Socket::ptr;

using Logger = mysylar::Logger;
using LoggerPtr = mysylar::Logger::ptr;

}
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace cmd {

class MsgPoint;
class MsgLoopframe;
struct DataBundle;
struct MessageContainer;

using MsgPointPtr = std::shared_ptr<MsgPoint>;
using MsgLoopframePtr = std::shared_ptr<MsgLoopframe>;
using DataBundlePtr = std::shared_ptr<DataBundle>;
using MessageContainerPtr = std::shared_ptr<MessageContainer>;
}


namespace cmd{

using int_t = uint32_t;
using precision_d = double;
using precision_f = float;
using precision_t = precision_d;


using MsgType = std::vector<uint32_t>;
using idpair = std::pair<int_t,int_t>;


// Eigen
template <int N>
using MatrixNType = Eigen::Matrix<precision_t, N, N>;
using Matrix3Type = MatrixNType<3>;
using Matrix4Type = MatrixNType<4>;
using Matrix6Type = MatrixNType<6>;
using Matrix7Type = MatrixNType<7>;

template <int N>
using VectorNType = Eigen::Matrix<precision_t, N,1>;
using Vec2Type = VectorNType<2>;
using Vec4Type = VectorNType<4>;

using Size = Vec2Type; // w h
using KType = Matrix3Type;
using TransMatrixType = Matrix4Type;
using TransQuatType = Eigen::Quaternion<precision_t>;


};