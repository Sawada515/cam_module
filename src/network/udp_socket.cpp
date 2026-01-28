/**
 * @file udp_sender.cpp
 * @brief UDP送信クラス
 * @author sawada
 * @date 2026-01-26
 */

#include "network/udp_socket.hpp"
#include "network/resolve_hostname.hpp"
#include "logger/logger.hpp"

#include <errno.h>

#include <string>
#include <cstring>
#include <cstdint>

#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>

UdpSocket::UdpSocket(const std::string& hostname, std::uint16_t port)
    : sock_fd_(-1)
{
    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        throw std::runtime_error("Failed to create socket");
    }

    std::string ip;

    try {
        ip = resolve_mdns_ipv4(hostname);
    }
    catch (const std::exception& e) {
        close(sock_fd_);

        throw std::runtime_error(std::string("Failed to resolve hostname") + std::string(e.what()));
    }

    std::memset(&dest_addr_, 0, sizeof(dest_addr_));

    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(port);

    if (::inet_pton(AF_INET, ip.c_str(), &dest_addr_.sin_addr) != 1) {
        ::close(sock_fd_);

        throw std::runtime_error("inet_pton failed");
    }

    int send_buf = 1024 * 1024;
    if (::setsockopt(sock_fd_, SOL_SOCKET, SO_SNDBUF, &send_buf, sizeof(send_buf)) < 0) {
        spdlog::warn("Failed to optimization send buffer");
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
        reinterpret_cast<const sockaddr*>(&dest_addr_),
        sizeof(dest_addr_)
    );

    if (send_bytes < 0) {
        return -1;  //呼び出し側でerrnoの確認
    }
    else {
        return send_bytes;
    }
}
