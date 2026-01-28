/**
 * @file udp_sender.hpp
 * @brief UDP送信クラス
 * @author sawada
 * @date 2026-01-26
 */

#ifndef UDP_SENDER_HPP_
#define UDP_SENDER_HPP_

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
         * @param[in] length 送信データのサイズ (MAX 1400程度) <= 呼び出し側で管理
         * @return -1 : errnoの確認, 0以上 送信済みバイト数
         */
        ssize_t send_raw_packet(const void *packet_ptr, size_t length);
    private:
        int sock_fd_ = -1;

        struct sockaddr_in dest_addr_{};
};

#endif
