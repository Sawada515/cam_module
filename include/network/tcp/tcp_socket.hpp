/**
 * @file tcp_socket.hpp
 * @brief TCP送受信クラス
 * @author sawada
 * @date 2026-01-26
 */

#include <string>
#include <cstring>
#include <cstdint>

#include <sys/uio.h>
#include <vector>
#include <deque>

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

        // /**
        //  * @brief データの送信
        //  * @param[in] packet_ptr 送信データの先頭アドレス
        //  * @param[in] length 送信データのサイズ
        //  * @return bool 0 : 成功 errno : 失敗
        //  */
        // send_state try_send(const void *packet_ptr, size_t length, ssize_t& sent_length);

        /**
         * @brief データの受信
         * @param[in] buffer 受信データ
         * @param[in] length 受信したいサイズ 
         * @param[out] received_length 受信したサイズ
         * @return 状態
         */
        recv_state try_recv(void *buffer, size_t length, ssize_t& received_length);

        /**
         * @brief 送信データをenqueue
         * @details データを内部queueにmoveする
         * @param[in] data 送信データ std::moveを使用
         */
        void enqueue_data(std::vector<std::uint8_t> data);

        /**
         * @brief enqueueされているデータをすべて送信
         * @details POLLOUT検出時に呼び出して溜まっているデータをsendmsgで送信
         * @return send_state
         */
        send_state send_enqueued_data();

        /**
         * @brief 内部queueのクリア
         */
        void clear_queue();

        /**
         * @brief 内部queueにデータがあるか
         */
        bool has_pending_data() const;

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

        /**
         * @brief ソケット ファイルディスクリプタの取得
         * @return ファイルディスクリプタ
         */
        int get_fd() const;
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

        std::deque<std::vector<std::uint8_t>> send_queue_;

        size_t current_packet_offset_ = 0;

        static constexpr int MAX_IOV_COUNT = 16;
};

#endif
