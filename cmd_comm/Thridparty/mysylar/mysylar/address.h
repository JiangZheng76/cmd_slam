#ifndef _SYLAR_ADDRESS_H_
#define _SYLAR_ADDRESS_H_
#include <memory>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <iostream>
#include <sys/un.h>
#include <vector>
#include <map>

namespace mysylar
{
    class IPAddress;

    class Address
    {
    public:
        typedef std::shared_ptr<Address> ptr;

        Address() {}
        virtual ~Address() {}

        int getFamily() const;

        static Address::ptr Create(sockaddr *addr);
        /**
         * @brief 作用是啥？？？寻找某一个域名或地址的addr
         * @param {string&} host
         * @param {int} family IPv4 / IPv6
         * @param {int} type tcp和udp，字节流
         * @param {int} protocol
         * @description:
         * @return {*}
         */
        static bool Lookup(std::vector<Address::ptr> &result, const std::string &host, int family = AF_UNSPEC, int type = 0, int protocol = 0);
        /**
         * @brief 对于 Lookup 的封装，寻找 IP 类型的地址，只要第一个返回的 IP 地址
         * @param {string&} host
         * @param {int} family
         * @param {int} type
         * @param {int} protocol
         * @description:
         * 利用 dynamic_pointer_cast 类型转换的方式来判断返回的是否为 IP 类型
         * @return {*}
         */
        static std::shared_ptr<IPAddress> LookupAnyIPAddress(const std::string &host, int family = AF_UNSPEC, int type = 0, int protocol = 0);
        /**
         * @brief 对于 Lookup 的封装，无论返回的是什么地址类型，只要第一个返回的地址
         * @param {string&} host
         * @param {int} family
         * @param {int} type
         * @param {int} protocol
         * @description:
         * @return {*}
         */
        static Address::ptr LookupAny(const std::string &host, int family = AF_UNSPEC, int type = 0, int protocol = 0);

        static bool GetInterafceAddress(std::multimap<std::string, std::pair<Address::ptr, uint32_t>> &result, int family);
        static bool GetInterafceAddress(std::vector<std::pair<Address::ptr, uint32_t>> &result, const std::string &iface, int family);

        virtual sockaddr *getAddr() const = 0;
        virtual socklen_t getAddrlen() const = 0;

        /// @brief ？？？这个有啥用？
        /// @param os
        /// @return
        virtual std::ostream &insert(std::ostream &os) const = 0;
        std::string toString();

        bool operator<(const Address &rhs) const;
        bool operator==(const Address &rhs) const;
        bool operator!=(const Address &rhs) const;
    };
    std::ostream& operator<<(std::ostream& os,const Address* addr);
    class IPAddress : public Address
    {
    public:
        typedef std::shared_ptr<IPAddress> ptr;

        static IPAddress::ptr Create(const char *address, uint16_t port);
        virtual IPAddress::ptr broadcastAddress(uint32_t prefix_len) = 0;
        virtual IPAddress::ptr networkAddress(uint32_t prefix_len) = 0;
        virtual IPAddress::ptr subnetMask(uint32_t prefix_len) = 0;

        // port的真实上限是16位
        virtual uint32_t getPort() const = 0;
        virtual void setPort(uint16_t v) = 0;
    };

    class IPv4Address : public IPAddress
    {
    public:
        typedef std::shared_ptr<IPv4Address> ptr;
        IPv4Address(uint32_t address = INADDR_ANY, uint16_t port = 0);
        IPv4Address(const sockaddr_in &addr);
        static IPv4Address::ptr Create(const char *address, uint16_t port);
        virtual sockaddr *getAddr() const override;
        virtual socklen_t getAddrlen() const override;
        virtual std::ostream &insert(std::ostream &os) const override;

        virtual IPAddress::ptr broadcastAddress(uint32_t prefix_len) override;
        virtual IPAddress::ptr networkAddress(uint32_t prefix_len) override;
        virtual IPAddress::ptr subnetMask(uint32_t prefix_len) override;
        virtual uint32_t getPort() const override;
        virtual void setPort(uint16_t v) override;

    private:
        sockaddr_in m_addr;
    };
    class IPv6Address : public IPAddress
    {
    public:
        typedef std::shared_ptr<IPv6Address> ptr;
        IPv6Address();
        IPv6Address(const char *address, uint16_t port = 0);
        IPv6Address(const sockaddr_in6 &addr);
        IPv6Address(const uint8_t address[16], uint16_t port);
        static IPv6Address::ptr Create(const char *address, uint16_t port);
        virtual sockaddr *getAddr() const override;
        virtual socklen_t getAddrlen() const override;
        virtual std::ostream &insert(std::ostream &os) const override;

        virtual IPAddress::ptr broadcastAddress(uint32_t prefix_len) override;
        virtual IPAddress::ptr networkAddress(uint32_t prefix_len) override;
        virtual IPAddress::ptr subnetMask(uint32_t prefix_len) override;
        virtual uint32_t getPort() const override;
        virtual void setPort(uint16_t v) override;

    private:
        sockaddr_in6 m_addr;
    };

    // ？？？这里面的定义不是很懂，可能是官方写法吧
    class UnixAdress : public Address
    {
    public:
        typedef std::shared_ptr<UnixAdress> ptr;
        UnixAdress(const std::string &path);
        UnixAdress();

        virtual sockaddr *getAddr() const override;
        virtual socklen_t getAddrlen() const override;
        virtual std::ostream &insert(std::ostream &os) const override;

        void setAddrlen(socklen_t len);

    private:
        struct sockaddr_un m_addr;
        // ？？？为什么IPv4他们不需要长度(因为 unix使用路径作为地址)
        //  地址的长度
        socklen_t m_length;
    };

    class UnkonwAddress : public Address
    {
    public:
        typedef std::shared_ptr<UnkonwAddress> ptr;
        UnkonwAddress(int family);
        UnkonwAddress(const sockaddr &addr);
        virtual sockaddr *getAddr() const override;
        virtual socklen_t getAddrlen() const override;
        virtual std::ostream &insert(std::ostream &os) const override;

    private:
        sockaddr m_addr;
    };


} // namespace mysylar

#endif