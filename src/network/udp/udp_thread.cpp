/**
 * @file udp_thread.cpp
 * @brief UDP通信スレッドクラス
 * @author sawada
 * @date 2026-01-30
 */

#include "network/udp/udp_thread.hpp"
#include "network/resolve_hostname.hpp"
#include "logger/logger.hpp"

#include <chrono>
#include <sys/uio.h>

UdpThread::UdpThread(std::string hostname, std::uint16_t port)
    : hostname_(std::move(hostname))
    , port_(port)
    , running_(false)
{
}

UdpThread::~UdpThread()
{
    stop();
}

void UdpThread::start()
{
    if (running_) {
        return;
    }

    running_ = true;
    worker_ = std::thread(&UdpThread::thread_loop, this);
}

void UdpThread::stop()
{
    if (!running_) {
        return;
    }

    running_ = false;

    if (worker_.joinable()) {
        worker_.join();
    }

    std::lock_guard<std::mutex> lock(mtx_);
    send_queue_.clear();
    socket_.reset();
}

bool UdpThread::send(const void* header, size_t header_len,
                     const void* payload, size_t payload_len)
{
    if (!running_) {
        return false;
    }

    SendItem item;
    item.header.assign(
        static_cast<const std::uint8_t*>(header),
        static_cast<const std::uint8_t*>(header) + header_len
    );
    item.payload.assign(
        static_cast<const std::uint8_t*>(payload),
        static_cast<const std::uint8_t*>(payload) + payload_len
    );

    std::lock_guard<std::mutex> lock(mtx_);
    send_queue_.push_back(std::move(item));
    return true;
}

void UdpThread::thread_loop()
{
    while (running_) {

        if (!socket_) {
            if (!resolve_and_create_socket()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
        }

        SendItem item;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (send_queue_.empty()) {
                goto sleep;
            }
            item = std::move(send_queue_.front());
            send_queue_.pop_front();
        }

        struct iovec iov[2];
        iov[0].iov_base = item.header.data();
        iov[0].iov_len  = item.header.size();
        iov[1].iov_base = item.payload.data();
        iov[1].iov_len  = item.payload.size();

        socket_->send_iovec(iov, 2);

    sleep:
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool UdpThread::resolve_and_create_socket()
{
    try {
        std::string ip = resolve_mdns_ipv4(hostname_);
        socket_ = std::make_unique<UdpSocket>(ip, port_);
        return true;
    }
    catch (const std::exception& e) {
        spdlog::warn("UDP resolve/connect failed: {}", e.what());
        socket_.reset();
        return false;
    }
}