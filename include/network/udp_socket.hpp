/**
 * @file udp_sender.hpp
 * @brief UDP送信クラス
 * @author sawada
 * @date 2026-01-26
 */

#ifndef UDP_SENDER_HPP_
#define UDP_SENDER_HPP_

#include <boost/asio.hpp>

#include <cstdint>
#include <string>
#include <cstring>

#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>

/**
 * @class UdpSocket
 * @brief UDPでデータを送信するだけのクラス
 */
class UdpSocket {
    public:
        UdpSocket(const std::string& hostname, std::uint16_t port);
        ~UdpSocket();

        /**
         * @brief UDPでデータを送信
         * @details 戻り値を確認
         * @param[in] packet_tpr 送信データの先頭アドレス
         * @param[in] length 送信データのサイズ (MAX 1400程度)
         * @return errno
         */
        int send_raw_packet(const void *packet_ptr, size_t length);
    private:
        int sock_fd_ = -1;
        struct sockaddr_in dest_addr_;
};

#endif
