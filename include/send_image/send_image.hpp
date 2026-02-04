/**
 * @file send_image.hpp
 * @brief 画像送信クラス
 * @author sawada
 * @date 2026-02-02
 * @details UDPを使用
 */

#ifndef SEND_IMAGE_HPP_
#define SEND_IMAGE_HPP_

#include <cstdint>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <condition_variable>

#include "network/udp/udp_socket.hpp"

/**
 * @brief UDPで画像を送信
 * @details 内部でヘッダを付与
 */
class SendImage
{
    public:
        SendImage(std::string hostname, std::uint16_t port);
        ~SendImage();

        SendImage(const SendImage&) = delete;
        SendImage& operator=(const SendImage&) = delete;

        /**
         * @brief スレッドの作成
         */
        void create_thread();

        /**
         * @details dataはstd::moveで渡す
         */
        bool set_send_data(std::vector<std::uint8_t>& jpeg_data, std::uint16_t width, std::uint16_t height, std::uint8_t channels);

    private:
        /**
         * @details Eth MAX mtu 1500, IP header 20byte UDP header 8byte
         */
        static constexpr size_t MAX_UDP_PAYLOAD_SIZE = 1472;
        /**
         * @details total header size: 15byte
         * @ref IMAGE_PACKET_HEADER_ANCHOR
         */
        static constexpr size_t IMAGE_HEADER_SIZE = 15;
        /**
         * @details 予備 7byte
         */
        static constexpr size_t PADDING_SIZE = 7;
        /**
         * @details NAX_UDP_PAYLOAD_SIZE - image_header(15byte) - padding(7byte)
         */
        static constexpr size_t MAX_IMAGE_PAYLOAD_SIZE = \
            MAX_UDP_PAYLOAD_SIZE - IMAGE_HEADER_SIZE - PADDING_SIZE;
        
        /**
         * @anchor IMAGE_PACKET_HEADER_ANCHOR
         * @brief image packet header
         * @details ネットワークバイトオーダ
         * @note uint8_t型の配列で送信する
         * @warning Do NOT define or send this header as a C/C++ struct.
         * @warning This header must be serialized manually into a byte array.
         * 
         * @par header format
         * Offset | Type | Size | Field
         * -------| ---- | ---- | ------
         * 0 | uint32_t | 4 | identifier 
         * 4 | uint16_t | 2 | packet_index
         * 6 | uint16_t | 2 | total_packet
         * 8 | uint16_t | 2 | data_size
         * 10 | uint16_t | 2 | width
         * 12 | uint16_t | 2 | height
         * 14 | uint8_t | 1 | channels
         */

        enum class buffer_state_t {
            EMPTY,
            FRONT_ONLY,
            FRONT_AND_BACK
        };

        static constexpr std::uint8_t OFFSET_IDENTIFIER = 0;
        static constexpr std::uint8_t OFFSET_PACKET_INDEX = 4;
        static constexpr std::uint8_t OFFSET_TOTAL_PACKET = 6;
        static constexpr std::uint8_t OFFSET_DATA_SIZE = 8;
        static constexpr std::uint8_t OFFSET_WIDTH = 10;
        static constexpr std::uint8_t OFFSET_HEIGHT = 12;
        static constexpr std::uint8_t OFFSET_CHANNELS = 14;

        void thread_loop();
    
        bool resolve_and_create_socket();

        void write_header_data_info_fields(std::uint8_t* h, std::uint16_t width, std::uint16_t height, std::uint8_t ch);

        void write_header_data_other_info(std::uint8_t* h, std::uint32_t identifier, std::uint16_t packet_index, std::uint16_t total_packet, std::uint16_t data_size);

        void recreate_socket();
    
        std::string hostname_;
        std::uint16_t port_;
    
        std::unique_ptr<UdpSocket> socket_;
    
        std::condition_variable cv_;
    
        std::thread worker_;
        std::mutex mtx_;
    
        std::atomic<bool> running_{false};
    
        bool is_created_thread_{false};

        enum buffer_state_t buffer_state_;

        std::vector<std::uint8_t> front_jpeg_data_;
        std::vector<std::uint8_t> back_jpeg_data_;

        std::uint32_t identifier_;

        std::uint8_t back_header_[IMAGE_HEADER_SIZE];
        std::uint8_t front_header_[IMAGE_HEADER_SIZE];
};

#endif
       