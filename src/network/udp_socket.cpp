/**
 * @file udp_sender.hpp
 * @brief UDP送信クラス
 * @author sawada
 * @date 2026-01-26
 */

#include "network/udp_socket.hpp"
#include "logger/logger.hpp"

#include <errno.h>

UdpSocket::UdpSocket(const std::string& hostname, std::uint16_t port)
    : sock_fd_(-1)
{
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        throw std::runtime_error("Failed to create socket");
    }

    struct addrinfo hints, *res;
    std::memset(&hints, 0, sizeof(hints));

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;

    if (getaddrinfo(hostname.c_str(), std::to_string(port).c_str(), &hints, &res) != 0) {
        close(sock_fd_);

        throw std::runtime_error("Failed to resolve hostname");
    }

    std::memcpy(&dest_addr_, res->ai_addr, sizeof(struct sockaddr_in));

    freeaddrinfo(res);

    int send_buf = 1024 * 1024;
    if (setsockopt(sock_fd_, SOL_SOCKET, SO_SNDBUF, &send_buf, sizeof(send_buf)) < 0) {
        spdlog::info("Failed to optimization send buffer");
    }
}

UdpSocket::~UdpSocket()
{
    if (sock_fd_ >= 0) {
        close(sock_fd_);
    }
}

int UdpSocket::send_raw_packet(const void *packet_ptr, size_t length)
{
    ssize_t send_bytes = sendto(sock_fd_, packet_ptr, length, 0,
        (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));

    if (send_bytes < 0) {
        return errno;
    }
    else {
        return 0;
    }
}
