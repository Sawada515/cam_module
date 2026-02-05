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
#include <poll.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "logger/logger.hpp"
#include "network/tcp/tcp_socket.hpp"

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

    int get_socket_error(int fd)
    {
        int err = 0;

        socklen_t err_len = sizeof(err);

        if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &err_len) < 0) {
            return errno;
        }

        return err;
    }
}

TcpSocket::TcpSocket(const std::string& ip_addr, std::uint16_t port)
    : sock_fd_(-1)
    , state_(connection_state::CLOSED)
    , last_error_(0)
    , current_packet_offset_(0)
{
    sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) {
        throw std::runtime_error("socket() failed");
    }

    try {
        set_non_blocking(sock_fd_);

        struct sockaddr_in addr;

        std::memset(&addr, 0, sizeof(addr));

        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);

        if (inet_pton(AF_INET, ip_addr.c_str(), &addr.sin_addr) <= 0) {
            throw std::runtime_error(std::string("Invalid IP address") + ip_addr);
        }

        state_ = connection_state::CONNECTING;

        int ret;
        do {
            ret = ::connect(sock_fd_, (struct sockaddr*)&addr, sizeof(addr));
        } while(ret < 0 && errno == EINTR);

        if (ret == 0) {
            state_ = connection_state::CONNECTED;
        }
        else if (errno == EINPROGRESS) {
            state_ = connection_state::CONNECTING;
        }
        else {
            last_error_ = errno;

            state_ = connection_state::ERROR;

            throw std::runtime_error("Failed to connect" + std::string(std::strerror(errno)));
        }
    }
    catch (...) {
        ::close(sock_fd_);

        sock_fd_ = -1;

        state_ = connection_state::CLOSED;

        throw;
    }
}

TcpSocket::~TcpSocket()
{
    if (sock_fd_ >= 0) {
        if (::close(sock_fd_) < 0) {
            spdlog::warn("Failed close sock_fd_: {}", std::string(std::strerror(errno)));
        }
    }

    state_ = connection_state::CLOSED;
}

short TcpSocket::poll_events() const
{
    switch (state_) {
        case connection_state::CONNECTING:
            return POLLOUT;
        case connection_state::CONNECTED: {
            short event = POLLIN;

            if (!send_queue_.empty()) {
                event |= POLLOUT;
            }

            return event;
        }
        case connection_state::PEER_CLOSED:
            return POLLIN;
        case connection_state::ERROR:
        case connection_state::CLOSED:
        default:
            return 0;
    }
}

void TcpSocket::handle_poll(short revents)
{
    if (state_ == connection_state::CLOSED || state_ == connection_state::ERROR) {
        return;
    }

    if (revents & POLLNVAL) {
        state_ = connection_state::ERROR;

        last_error_ = EBADF;

        spdlog::error("Poll returned POLLNVAL");

        return;
    }

    if (revents & (POLLERR | POLLHUP)) {
        int err = get_socket_error(sock_fd_);
        if (err != 0) {
            state_ = connection_state::ERROR;

            last_error_ = err;

            return;
        }
        else {
            last_error_ = ECONNRESET;
        }

        state_ = connection_state::ERROR;

        clear_queue();

        return;
    }

    if (state_ == connection_state::CONNECTING) {
        if (revents & (POLLOUT | POLLIN)) {
            int err = get_socket_error(sock_fd_);
            if (err == 0) {
                state_ = connection_state::CONNECTED;
            }
            else {
                state_ = connection_state::ERROR;

                last_error_ = err;
            }
        }
    }
}

/*
TcpSocket::send_state TcpSocket::try_send(const void *packet_ptr, size_t length, ssize_t& sent_length)
{
    sent_length = 0;

    if (state_ != connection_state::CONNECTED && state_ != connection_state::PEER_CLOSED) {
        return send_state::ERROR;
    }

    ssize_t ret = ::send(sock_fd_, packet_ptr, length, MSG_NOSIGNAL);
    if (ret < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return send_state::TRY_LATER;
        }

        last_error_ = errno;
        if (errno == EPIPE || errno == ECONNRESET) {
            state_ = connection_state::ERROR;

            return send_state::CLOSED;
        }

        state_ = connection_state::ERROR;

        return send_state::ERROR;
    }

    sent_length = ret;

    return send_state::DATA_SENT;
}
*/

TcpSocket::recv_state TcpSocket::try_recv(void *buffer, size_t length, ssize_t& received_length)
{
    received_length = 0;

    if (state_ != connection_state::CONNECTED) {
        if (state_ == connection_state::PEER_CLOSED) {
            return recv_state::CLOSED;
        }

        return recv_state::ERROR;
    }

    ssize_t ret = ::recv(sock_fd_, buffer, length, 0);
    if (ret < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return recv_state::TRY_LATER;
        }

        last_error_ = errno;

        state_ = connection_state::ERROR;

        return recv_state::ERROR;
    }

    if (ret == 0) {
        state_ = connection_state::PEER_CLOSED;

        return recv_state::CLOSED;
    }
    
    received_length = ret;

    return recv_state::DATA_RECEIVED;
}

void TcpSocket::enqueue_data(std::vector<std::uint8_t> data)
{
    if (data.empty()) {
        return;
    }

    send_queue_.push_back(std::move(data));
}

TcpSocket::send_state TcpSocket::send_enqueued_data() 
{
    if (state_ == connection_state::CONNECTING) {
        return send_state::TRY_LATER;
    }

    if (state_ != connection_state::CONNECTED) {
        return send_state::CLOSED;
    }

    if (send_queue_.empty()) {
        return send_state::DATA_SENT;
    }

    struct iovec iov[MAX_IOV_COUNT];
    int iov_cnt = 0;

    for (const auto& pkt : send_queue_) {
        if (iov_cnt >= MAX_IOV_COUNT) {
            break;
        }

        if (iov_cnt == 0) {
            iov[0].iov_base = const_cast<std::uint8_t*>(pkt.data() + current_packet_offset_);
            iov[0].iov_len  = pkt.size() - current_packet_offset_;
        } else {
            iov[iov_cnt].iov_base = const_cast<std::uint8_t*>(pkt.data());
            iov[iov_cnt].iov_len  = pkt.size();
        }

        iov_cnt += 1;
    }

    struct msghdr msg = {};
    msg.msg_iov = iov;
    msg.msg_iovlen = iov_cnt;

    ssize_t sent_bytes;

    do {
        sent_bytes = ::sendmsg(sock_fd_, &msg, MSG_NOSIGNAL);
    } while (sent_bytes < 0 && errno == EINTR);

    if (sent_bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return send_state::TRY_LATER;
        }
        
        last_error_ = errno;
        
        if (errno == EPIPE || errno == ECONNRESET) {
            state_ = connection_state::ERROR; 

            clear_queue();

            return send_state::CLOSED;
        }

        state_ = connection_state::ERROR;

        clear_queue();

        return send_state::ERROR;
    }

    size_t remaining = static_cast<size_t>(sent_bytes);

    while (remaining > 0 && !send_queue_.empty()) {
        auto& front_pkt = send_queue_.front();
        size_t front_rem_size = front_pkt.size() - current_packet_offset_;

        if (remaining >= front_rem_size) {
            remaining -= front_rem_size;

            send_queue_.pop_front();

            current_packet_offset_ = 0;
        } else {
            current_packet_offset_ += remaining;

            remaining = 0;
        }
    }

    return send_state::DATA_SENT;
}

void TcpSocket::clear_queue()
{
    send_queue_.clear();
    current_packet_offset_ = 0;
}

bool TcpSocket::has_pending_data() const
{
    return !send_queue_.empty();
}

bool TcpSocket::is_connected() const
{
    return state_ == connection_state::CONNECTED;
}

bool TcpSocket::is_connecting() const
{
    return state_ == connection_state::CONNECTING;
}

bool TcpSocket::can_send() const
{
    return state_ == connection_state::CONNECTED;
}

bool TcpSocket::can_recv() const
{
    return state_ == connection_state::CONNECTED;
}

int TcpSocket::get_fd() const
{
    return sock_fd_;
}