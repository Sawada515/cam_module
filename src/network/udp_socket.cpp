/**
 * @file udp_socket.cpp
 * @brief UDP送信クラス
 * @author sawada
 * @date 2026-01-26
 */

#include "network/udp_socket.hpp"
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
{
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        throw std::runtime_error("Failed to create socket");
    }


    std::memset(&addr_, 0, sizeof(addr_));

    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(port);

    if (inet_pton(AF_INET, ip_addr.c_str(), &addr_.sin_addr) <= 0) {
        ::close(sock_fd_);

        sock_fd_ = -1;

        throw std::runtime_error(std::string("Invalid IP address") + ip_addr);
    }

    int send_buf_size = 1024 * 1024;
    if (::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size)) < 0) {
        spdlog::warn("Failed to optimize send buffer");
    }
}

UdpSocket::~UdpSocket()
{
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
    }
}

ssize_t UdpSocket::send_raw_packet(const void *packet_ptr, size_t length)
{
    ssize_t send_bytes = ::sendto(
        sock_fd_, 
        packet_ptr, 
        length,
        0,
        reinterpret_cast<const sockaddr*>(&addr_),
        sizeof(addr_)
    );

    if (send_bytes < 0) {
        return -1;  //呼び出し側でerrnoの確認
    }

    return send_bytes;
}
