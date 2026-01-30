/**
 * @file json_client.hpp
 * @brief JSON Client wrapper
 * @author sawada
 * @date 2026-01-30
 */

#ifndef JSON_CLIENT_HPP_
#define JSON_CLIENT_HPP_

#include <string>
#include <vector>
#include <optional>
#include <cstdint>

#include "json/json_codec.hpp"
#include "network/tcp/tcp_thread.hpp"

#include "camera/v4l2_capture.hpp"

/**
 * @class JsonProtocolClient
 * @brief TCP通信とJSON変換をラップし、コマンド・データの送受信を行うクラス
 * @note 送信時に4byte(Big Endian)のサイズヘッダを付与する
 */
class JsonClient
{
    public:
        enum class cmd_kinds {
            CHANGE_FORMAT,
            SHUTDOWN,
            UNKNOWN
        };

        enum class args_kinds : std::uint32_t {
            NONE = 0,
            MJPEG = static_cast<std::uint32_t>(V4L2Capture::frame_format::MJPEG),
            YUV422 = static_cast<std::uint32_t>(V4L2Capture::frame_format::YUV422)
        };
        struct recv_cmd_data {
            cmd_kinds cmd;
            args_kinds args;
        };

        JsonClient(const std::string& hostname, std::uint16_t port);
        ~JsonClient();
    
        /**
         * @brief 通信スレッドの開始
         */
        void start();
    
        /**
         * @brief 通信スレッドの停止
         */
        void stop();
    
        /**
         * @brief コマンドパケットの送信
         * @param cmd 送信するコマンド構造体
         */
        void send_command(const type_cmd_json& cmd);
    
        /**
         * @brief データパケットの送信
         * @param data 送信するデータ構造体
         */
        void send_data(const type_data_json& data);
    
        /**
         * @brief パケットの受信（ノンブロッキング）
         * @return 受信データがあればVariantを返す。なければ std::nullopt
         */
        std::optional<recv_cmd_data> try_receive();
    
    private:
        /**
         * @brief JSON文字列に4byteヘッダを付与してバイナリ化
         */
        std::vector<std::uint8_t> add_header_and_bytes(const std::string& str) const;
    
        /**
         * @brief バイナリ配列を文字列に変換
         */
        std::string bytes_to_string(const std::vector<std::uint8_t>& bytes) const;

        /**
         * @brief json_strからrecv_cmd_dataへの変換
         */
        std::optional<recv_cmd_data> json_str_to_recv_cmd_data(std::string json_str);
    
        TcpThread tcp_thread_;
};

#endif