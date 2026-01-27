/**
 * @file tcp_socket.cpp
 * @brief TCP送受信クラス
 * @author sawada
 * @date 2026-01-27
 */

#include <string>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <errno.h>

#include "logger/logger.hpp"
#include "network/tcp_socket.hpp"

TcpSocket::TcpSocket(const std::string& hostname, std::uint16_t port)
    : sock_fd_(-1)
    , is_connected_(false)
{
    sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) {
        throw std::runtime_error("Failed to create socket");
    }

    set_non_blocking(sock_fd_);

    struct addrinfo hints, *res;
    std::memset(&hints, 0, sizeof(hints));

    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo(hostname.c_str(), std::to_string(port).c_str(), &hints, &res) != 0) {
        close(sock_fd_);

        throw std::runtime_error("Failed to resolve hostname : " + hostname);
    }

    int ret = connect(sock_fd_, res->ai_addr, res->ai_addrlen);

    freeaddrinfo(res);

    if (ret < 0) {
        if (errno != EINPROGRESS) {
            close(sock_fd_);

            spdlog::error("Connection failed : {}", std::string(strerror(errno)));

            throw std::runtime_error("Connection failed : " + std::string(strerror(errno)));
        }

        struct pollfd poll_fd;
        poll_fd.fd = sock_fd_;
        poll_fd.events = POLLOUT;

        int poll_ret = poll(&poll_fd, 1, 3000);
        if (poll_ret == 0) {
            close(sock_fd_);

            throw std::runtime_error("Connection timeout");
        }
        else if (poll_ret < 0) {
            close(sock_fd_);

            throw std::runtime_error("poll failed during connection");
        }

        int so_error = 0;
        socklen_t sock_len = sizeof(so_error);
        if (getsockopt(sock_fd_, SOL_SOCKET, SO_ERROR, &so_error, &sock_len) < 0 || so_error != 0) {
            close(sock_fd_);

            throw std::runtime_error("Connection failed : " + std::string(strerror(so_error)));
        }
    }

    is_connected_ = true;
}

TcpSocket::~TcpSocket()
{
    if (sock_fd_ >= 0) {
        close(sock_fd_);
    }
}

ssize_t TcpSocket::send_raw_packet(const void *packet_ptr, size_t length)
{
    if (!is_connected_) {
        errno = ENOTCONN;

        return -1;
    }

    ssize_t send_bytes = send(sock_fd_, packet_ptr, length, MSG_NOSIGNAL);

    if (send_bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;   //再試行させるため
        }
        else if (errno == EINTR) {
            return 0;
        }

        is_connected_ = false;

        return -1;
    }

    return send_bytes;
}

ssize_t TcpSocket::receive_data(void *buffer, size_t length)
{
    if (!is_connected_) {
        return -1;
    }

    ssize_t receive_bytes = recv(sock_fd_, buffer, length, 0);

    if (receive_bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {

            return 0; 
        }
        is_connected_ = false;

        return -1;
    }
    else if (receive_bytes  == 0) {
        is_connected_ = false;

        return -1;
    }

    return receive_bytes ;
}

bool TcpSocket::is_connected() const
{
    return is_connected_;
}

void TcpSocket::set_non_blocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
}
