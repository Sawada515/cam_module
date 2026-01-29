/**
 * @file tcp_thread.hpp
 * @brief TCP thread クラス
 * @author sawada
 * @date 2026-01-29
 */

#ifndef TCP_THREAD_HPP_
#define TCP_THREAD_HPP_

#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <memory>
#include <string>
#include <deque>

#include "network/tcp_socket.hpp"
#include "network/resolve_hostname.hpp"

/**
 * @class TcpThread
 * @brief TCPの送受信をスレッドで行う
 */
class TcpThread {
public:
    TcpThread(std::string hostname, std::uint16_t port);
    ~TcpThread();

    /**
     * @brief スレッドの開始 
     */
    void start();

    /**
     * @brief スレッドの停止
     */
    void stop();

    /**
     * @brief スレッドを起こしてデータの送信
     */
    void send(std::vector<std::uint8_t> data);

    /**
     * @brief 受信データがあるか
     */
    bool has_received_data() const;

    /**
     * @brief 受信データの受け取り
     */
    bool recv(std::vector<std::uint8_t>& out);

private:
    void thread_loop();
    bool resolve_and_connect();

    std::string hostname_;
    std::uint16_t port_;

    std::unique_ptr<TcpSocket> socket_;

    std::thread worker_;
    std::atomic<bool> running_;

    mutable std::mutex mtx_;
    std::condition_variable cv_;

    std::deque<std::vector<std::uint8_t>> recv_queue_;

    std::deque<std::vector<std::uint8_t>> thread_send_queue_;
};

#endif
