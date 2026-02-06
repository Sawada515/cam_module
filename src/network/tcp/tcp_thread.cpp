/**
 * @file tcp_thread.cpp
 * @brief TCPスレッドクラスの実装
 * @author sawada
 * @date 2026-01-29
 */

#include "network/tcp/tcp_thread.hpp"
#include "network/tcp/tcp_socket.hpp"
#include "network/resolve_hostname.hpp"

#include "logger/logger.hpp"

#include <chrono>
#include <cerrno>

#include <poll.h>
#include <unistd.h>
#include <arpa/inet.h>

namespace {
    constexpr size_t RECV_BUFFER_DEFAULT_SIZE = 2048;
}

TcpThread::TcpThread(std::string hostname, std::uint16_t port)
    : hostname_(std::move(hostname))
    , port_(port)
    , running_(false)
{
    recv_buffer_.resize(RECV_BUFFER_DEFAULT_SIZE);
}

TcpThread::~TcpThread()
{
    stop();
}

void TcpThread::start()
{
    if (running_) {
        return;
    }

    running_ = true;
    worker_ = std::thread(&TcpThread::thread_loop, this);
}

void TcpThread::stop()
{
    if (!running_) {
        return;
    }

    running_ = false;
    cv_.notify_all();

    if (worker_.joinable()) {
        worker_.join();
    }

    std::lock_guard<std::mutex> lock(mtx_);
    recv_buffer_.clear();
    socket_.reset();
}

void TcpThread::send(std::vector<std::uint8_t> data)
{
    if (data.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mtx_);

    thread_send_queue_.push_back(std::move(data));
}

bool TcpThread::has_received_data() const
{
    std::lock_guard<std::mutex> lock(mtx_);

    return !recv_buffer_.empty();
}

bool TcpThread::fetch_recv_data(std::vector<std::uint8_t>& out)
{
    std::lock_guard<std::mutex> lock(mtx_);

    if (recv_buffer_.empty()) {
        return false;
    }

    if (out.empty()) {
        std::swap(out, recv_buffer_);
    }
    else {
        out.reserve(out.size() + recv_buffer_.size());

        out.insert(out.end(), recv_buffer_.begin(), recv_buffer_.end());

        recv_buffer_.clear();
    }

    return true;
}

void TcpThread::thread_loop()
{
    while (running_) {
        if (!socket_ || !socket_->is_connected()) {
            if (!resolve_and_connect()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));

                continue;
            }

            recv_buffer_.clear();
        }

        {
            std::lock_guard<std::mutex> lock(mtx_);

            while (!thread_send_queue_.empty()) {
                if (socket_) {
                    socket_->enqueue_data(std::move(thread_send_queue_.front()));
                }

                thread_send_queue_.pop_front();
            }
        }

        struct pollfd pfd {};
        pfd.fd = socket_->get_fd();
        pfd.events = socket_->poll_events();

        int ret = ::poll(&pfd, 1, 10);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            socket_.reset();
            continue;
        }

        if (ret == 0) {
            continue;
        }

        socket_->handle_poll(pfd.revents);

        if (pfd.revents & POLLIN) {
            std::uint8_t buf[2048];
            ssize_t recv_length;

            while (1) {
                TcpSocket::recv_state state = socket_->try_recv(buf, sizeof(buf), recv_length);
                
                if (state == TcpSocket::recv_state::DATA_RECEIVED && recv_length > 0) {
                    {
                        std::lock_guard<std::mutex> lock(mtx_);

                        recv_buffer_.insert(recv_buffer_.end(), buf, buf + recv_length);
                    }
                }
                else if (state == TcpSocket::recv_state::CLOSED) {
                    spdlog::warn("TcpThread: Connection closed by peer");

                    socket_.reset();

                    break;
                }
                else {
                    break;
                }
            }
        }

        if ((pfd.revents & POLLOUT) && socket_ && socket_->is_connected()) {
            TcpSocket::send_state state;

            while (1) {
                state = socket_->send_enqueued_data();

                if (state == TcpSocket::send_state::TRY_LATER) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(25));

                    continue;
                }
                else if (state == TcpSocket::send_state::CLOSED || state == TcpSocket::send_state::ERROR) {
                    spdlog::warn("TcpThread; Connectin closed");

                    socket_.reset();

                    break;
                }
                else {
                    break;
                }
            }
        }
    }
}

bool TcpThread::resolve_and_connect()
{
    try {
        std::string ip = resolve_mdns_ipv4(hostname_);

        socket_ = std::make_unique<TcpSocket>(ip, port_);

        return true;
    }
    catch (...) {
        socket_.reset();

        return false;
    }
}
