#pragma once

#include <memory>
#include <vector>
#include <list>
#include <sstream>

// thrid-party
// #include "Eigen/Core"
#include "sophus/se3.hpp"
// sylar
#include <log.h>
#include <address.h>
#include <bytearray.h>
#include <socket_stream.h>
#include <thread.hh>
// using namespace mysylar;
namespace cmd
{
    using namespace mysylar;
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

    using RWMutextType = mysylar::RWMutex;
    using MutextType = mysylar::Mutex;

    using Thread = mysylar::Thread;
    using ThreadPtr = mysylar::Thread::ptr;

}
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace cmd
{

    class MsgPoint;
    class MsgLoopframe;
    struct DataBundle;
    struct MessageContainer;

    using MsgLoopframePtr = std::shared_ptr<MsgLoopframe>;
    using DataBundlePtr = std::shared_ptr<DataBundle>;
    using MessageContainerPtr = std::shared_ptr<MessageContainer>;
}

namespace cmd
{

    using int_t = uint32_t;
    using precision_d = double;
    using precision_f = float;
    using precision_t = precision_d;

    using MsgType = std::vector<uint32_t>; // [msg 长度, SentOnce, id.first, id.second，类型（lf or point）]
    using idpair = std::pair<int_t, int_t>;

    // Eigen
    template <int N>
    using MatrixNType = Eigen::Matrix<precision_t, N, N>;
    using Matrix3Type = MatrixNType<3>;
    using Matrix4Type = MatrixNType<4>;
    using Matrix6Type = MatrixNType<6>;
    using Matrix7Type = MatrixNType<7>;

    template <int N>
    using VectorNType = Eigen::Matrix<precision_t, N, 1>;
    using Vec2Type = VectorNType<2>;
    using Vec3Type = VectorNType<3>;
    using Vec3bType = Eigen::Matrix<unsigned char,3,1>;
    using Vec4Type = VectorNType<4>;

    using Size = Vec2Type; // w h
    using KType = Matrix3Type;

    using TransMatrixType = Sophus::SE3d;
    

    using RotateMatrixType = Sophus::SO3d;

    using TransQuatType = precision_t[6];

};