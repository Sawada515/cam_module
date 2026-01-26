/**
 * @file tcp_socket.hpp
 * @brief TCP送受信クラス
 * @author sawada
 * @date 2026-01-26
 */

#include <string>
#include <cstring>
#include <cstdint>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#ifndef TCP_SOCKET_HPP_
#define TCP_SOCKET_HPP_

/**
 * @class TcpSocket
 * @brief TCPで双方向通信するためのクラス(ノンブロッキング)
 */
class TcpSocket {
    public:
        TcpSocket(const std::string& hostname, std::uint16_t port);
        ~TcpSocket();

        /**
         * @brief データの送信
         * @param[in] packet_ptr 送信データの先頭アドレス
         * @param[in] length 送信データのサイズ
         * @return bool 0 : 成功 errno : 失敗
         */
        int send_raw_packet(const void *packet_ptr, size_t length);

        /**
         * @brief データの受信
         * @param[out] buffer 受信データ
         * @param[in] length 受信したいサイズ 
         * @return 実際に受信したサイズ(0 : disconnected, -1 : error)
         */
        ssize_t receive_data(void *buffer, size_t length);

        /**
         * @brief 接続状況の確認
         * @return bool true : 接続, false : 接続断
         */
        bool is_connected() const;
    private:
        int sock_fd_ = -1;
        bool is_connected_ = false;

        void set_non_blocking(int fd);
};

#endif
