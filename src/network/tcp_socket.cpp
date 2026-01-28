/**
 * @file tcp_socket.cpp
 * @brief TCP送受信クラス
 * @author sawada
 * @date 2026-01-27
 */

#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <poll.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "logger/logger.hpp"
#include "network/tcp_socket.hpp"
#include "network/resolve_hostname.hpp"

namespace {
    void set_non_blocking(int fd)
    {
        int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags < 0) {
            throw std::runtime_error("fcntl(F_FETFL) failed");
        }

        if (::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
            throw std::runtime_error("fcntl(F_SETFL) failed");
        }
    }
}

TcpSocket::TcpSocket(const std::string& hostname, std::uint16_t port)
    : sock_fd_(-1)
    , alive_(false)
    , connecting_(true)
{
    sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) {
        throw std::runtime_error("socket() failed");
    }

    try {
        set_non_blocking(sock_fd_);

        struct addrinfo hints{};
        struct addrinfo* res = nullptr;

        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;


        const std::string service = std::to_string(port);
        int ret = ::getaddrinfo(hostname.c_str(), service.c_str(), &hints, &res);
        if (ret != 0) {
            throw std::runtime_error("getaddrinfo failed");
        }

        ret = ::connect(sock_fd_, res->ai_addr, res->ai_addrlen);

        ::freeaddrinfo(res);

        if (ret < 0 && errno != EINPROGRESS) {
            throw std::runtime_error(std::string("connect failed: ") + std::strerror(errno));
        }
    }
    catch (...) {
        ::close(sock_fd_);

        sock_fd_ = -1;
        alive_ = false;

        throw;
    }
}


TcpSocket::~TcpSocket()
{
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
    }

    alive_ = false;
    connecting_ = false;
}


short TcpSocket::poll_events() const
{
    if (!alive_ && !connecting_) {
        return 0;
    }

    short ev = POLLERR | POLLHUP | POLLNVAL;

    if (connecting_) {
        ev |= POLLOUT;
    }
    else {
        ev |= POLLIN | POLLOUT;
    }

    return ev;
}


void TcpSocket::handle_poll(short revents)
{
    if (revents & (POLLERR | POLLHUP | POLLNVAL)) {
        alive_ = false;

        return;
    }

    if (connecting_ && (revents & POLLOUT)) {
        int err = 0;

        socklen_t len = sizeof(err);

        if (::getsockopt(sock_fd_, SOL_SOCKET, SO_ERROR, &err, &len) < 0 || err != 0) {
            alive_ = false;

            connecting_ = false;
        }
        else {
            connecting_ = false;

            alive_ =  true;
        }
    }
}

TcpSocket::send_state TcpSocket::try_send(const void *packet_ptr, size_t length, ssize_t& sent_length)
{
    if (!alive_) {
        errno = ENOTCONN;

        return send_state::ERROR;
    }

    sent_length = 0;

    ssize_t ret = ::send(sock_fd_, packet_ptr, length, MSG_NOSIGNAL);
    if (ret < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return send_state::TRY_LATER;
        }

        alive_ = false;

        return send_state::CLOSED;
    }

    sent_length = ret;

    return send_state::DATA_SENT;
}


TcpSocket::recv_state TcpSocket::try_recv(void *buffer, size_t length, ssize_t& received_length)
{
    if (!alive_) {
        errno = ENOTCONN;

        return recv_state::ERROR;
    }


    received_length = 0;

    ssize_t ret = ::recv(sock_fd_, buffer, length, 0);
    if (ret < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return recv_state::TRY_LATER;
        }

        alive_ = false;

        return recv_state::ERROR;
    }

    if (ret == 0) {
        alive_ = false;

        return recv_state::CLOSED;
    }
    
    received_length = ret;

    return recv_state::DATA_RECEIVED;
}


bool TcpSocket::alive() const
{
    return alive_;
}
