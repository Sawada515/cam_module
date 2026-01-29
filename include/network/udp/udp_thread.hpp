/**
 * @file udp_thread.hpp
 * @brief UDPスレッド通信クラス
 * @author sawada
 * @date 2026-01-30
 */

#ifndef UDP_THREAD_HPP_
#define UDP_THREAD_HPP_

#include <cstdint>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <string>

#include "network/udp/udp_socket.hpp"

/**
 * @brief UDP通信スレッドクラス
 */
class UdpThread
{
public:
    UdpThread(std::string hostname, std::uint16_t port);
    ~UdpThread();

    UdpThread(const UdpThread&) = delete;
    UdpThread& operator=(const UdpThread&) = delete;

    void start();
    void stop();

    bool send(const void* header, size_t header_len,
              const void* payload, size_t payload_len);

private:
    struct SendItem {
        std::vector<std::uint8_t> header;
        std::vector<std::uint8_t> payload;
    };

    void thread_loop();
    bool resolve_and_create_socket();

    std::string hostname_;
    std::uint16_t port_;

    std::unique_ptr<UdpSocket> socket_;

    std::thread worker_;
    std::mutex mtx_;
    std::deque<SendItem> send_queue_;

    std::atomic<bool> running_;
};

#endif
