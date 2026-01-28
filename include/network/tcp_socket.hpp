/**
 * @file tcp_socket.hpp
 * @brief TCP送受信クラス
 * @author sawada
 * @date 2026-01-26
 */

#include <string>
#include <cstring>
#include <cstdint>

#ifndef TCP_SOCKET_HPP_
#define TCP_SOCKET_HPP_

/**
 * @class TcpSocket
 * @brief TCPで双方向通信するためのクラス(ノンブロッキング)
 */
class TcpSocket {
    public:
        enum class recv_state : std::uint8_t {
            DATA_RECEIVED,
            TRY_LATER,
            CLOSED,
            ERROR
        };

        enum class send_state : std::uint8_t {
            DATA_SENT,
            TRY_LATER,
            CLOSED,
            ERROR
        };

        TcpSocket(const std::string& ip_addr, std::uint16_t port);
        ~TcpSocket();

        /**
         * @brief 監視するイベントの設定
         * @return POLLERR, POLLHUP, POLLNVAL
         */
        short poll_events() const;

        /**
         * @イベントを
         */
        void handle_poll(short revents);

        /**
         * @brief データの送信
         * @param[in] packet_ptr 送信データの先頭アドレス
         * @param[in] length 送信データのサイズ
         * @return bool 0 : 成功 errno : 失敗
         */
        send_state try_send(const void *packet_ptr, size_t length, ssize_t& sent_length);

        /**
         * @brief データの受信
         * @param[in] buffer 受信データ
         * @param[in] length 受信したいサイズ 
         * @param[out] received_length 受信したサイズ
         * @return 状態
         */
        recv_state try_recv(void *buffer, size_t length, ssize_t& received_length);

        /**
         * @brief 接続状況の確認
         * @return bool true : 接続, false : 接続断
         */
        bool is_connected() const;

        /**
         * @brief 接続状況の確認
         * @return bool true : 接続, false : 接続断
         */
        bool is_connecting() const;

        /**
         * @brief 送信可能か
         * @return bool 
         */
        bool can_send() const;

        /**
         * @brief 受信可能か
         * @return bool
         */
        bool can_recv() const;
    private:
        enum class connection_state : std::uint8_t {
            CONNECTING,
            CONNECTED,
            PEER_CLOSED,
            ERROR,
            CLOSED
        };
        int sock_fd_;

        connection_state state_;
        int last_error_; 
};

#endif
