/**
 * @file udp_socket.cpp
 * @brief UDP送信クラス
 * @author sawada
 * @date 2026-01-26
 */

#include "network/udp/udp_socket.hpp"
#include "logger/logger.hpp"

#include <errno.h>

#include <string>
#include <cstring>
#include <cstdint>

#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>

UdpSocket::UdpSocket(const std::string& ip_addr, std::uint16_t port)
    : sock_fd_(-1)
    , ip_addr_(ip_addr)
    , port_(port)
{
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        throw std::runtime_error("Failed to create socket");
    }

    std::memset(&addr_, 0, sizeof(addr_));

    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(port_);

    if (inet_pton(AF_INET, ip_addr_.c_str(), &addr_.sin_addr) <= 0) {
        ::close(sock_fd_);

        sock_fd_ = -1;

        throw std::runtime_error(std::string("Invalid IP address") + ip_addr_);
    }

    int send_buf_size = 1024 * 1024;
    if (::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size)) < 0) {
        spdlog::warn("Failed to optimize send buffer");
    }
}

UdpSocket::UdpSocket(UdpSocket&& other) noexcept
{
    sock_fd_ = other.sock_fd_;
    addr_    = other.addr_;
    other.sock_fd_ = -1;
}

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept
{
    if (this != &other) {
        if (sock_fd_ >= 0) {
            ::close(sock_fd_);
        }
        sock_fd_ = other.sock_fd_;
        addr_    = other.addr_;
        other.sock_fd_ = -1;
    }
    return *this;
}

ssize_t UdpSocket::send_iovec(const struct iovec* iov, size_t iov_size) noexcept
{
    if (!iov || iov_size <= 0) {
        errno = EINVAL;
        return -1;
    }

    struct msghdr msg {};
    msg.msg_name    = &addr_;
    msg.msg_namelen = sizeof(addr_);
    msg.msg_iov     = const_cast<struct iovec*>(iov);
    msg.msg_iovlen  = iov_size;

    return ::sendmsg(sock_fd_, &msg, 0);
}

int UdpSocket::get_fd() const noexcept
{
    return sock_fd_;
}
