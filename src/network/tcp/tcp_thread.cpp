/**
 * @file tcp_thread.cpp
 * @brief TCPスレッドクラスの実装
 * @author sawada
 * @date 2026-01-29
 */

#include "network/tcp/tcp_thread.hpp"
#include "network/tcp/tcp_socket.hpp"
#include "network/resolve_hostname.hpp"

#include <poll.h>
#include <unistd.h>
#include <chrono>
#include <cerrno>

TcpThread::TcpThread(std::string hostname, std::uint16_t port)
    : hostname_(std::move(hostname))
    , port_(port)
    , running_(false)
{
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
    recv_queue_.clear();
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
    return !recv_queue_.empty();
}

bool TcpThread::recv(std::vector<std::uint8_t>& out)
{
    std::lock_guard<std::mutex> lock(mtx_);

    if (recv_queue_.empty()) {
        return false;
    }

    out = std::move(recv_queue_.front());
    recv_queue_.pop_front();
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
            std::uint8_t buf[4096];
            ssize_t len;

            while (true) {
                auto st = socket_->try_recv(buf, sizeof(buf), len);
                if (st != TcpSocket::recv_state::DATA_RECEIVED) {
                    break;
                }

                std::vector<std::uint8_t> pkt(buf, buf + len);
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    recv_queue_.push_back(std::move(pkt));
                }
            }
        }

        if ((pfd.revents & POLLOUT) && socket_ && socket_->is_connected()) {
            socket_->send_enqueued_data();
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
