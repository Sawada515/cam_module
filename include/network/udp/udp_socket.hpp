/**
 * @file udp_socket.hpp
 * @brief UDP送信クラス
 * @author sawada
 * @date 2026-01-26
 */

#ifndef UDP_SENDER_HPP_
#define UDP_SENDER_HPP_

#include <cstdint>
#include <cstddef>
#include <string>

#include <sys/uio.h>
#include <netinet/in.h>

/**
 * @class UdpSocket
 * @brief UDPでデータを送信するだけのクラス
 */
class UdpSocket {
    public:
        UdpSocket(const std::string& ip_addr, std::uint16_t port);
        ~UdpSocket();

        UdpSocket(const UdpSocket&) = delete;
        UdpSocket& operator=(const UdpSocket&) = delete;

        UdpSocket(UdpSocket&& other) noexcept;
        UdpSocket& operator=(UdpSocket&& other) noexcept;

        /**
         * @brief UDPでデータを送信
         * @details 戻り値を確認
         * @return -1 : errnoの確認, 0以上 送信済みバイト数
         */
        ssize_t send_iovec(const struct iovec* iov, int iov_count) noexcept;

        int get_fd() const noexcept;
    private:
        int sock_fd_ = -1;

        struct sockaddr_in addr_{};
};

#endif
