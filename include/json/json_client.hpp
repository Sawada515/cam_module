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

/**
 * @class JsonProtocolClient
 * @brief TCP通信とJSON変換をラップし、コマンド・データの送受信を行うクラス
 * @note 送信時に4byte(Big Endian)のサイズヘッダを付与する
 */
class JsonClient
{
    public:
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
        std::optional<JsonCodec::PacketVariant> try_receive();
    
    private:
        /**
         * @brief JSON文字列に4byteヘッダを付与してバイナリ化
         */
        std::vector<std::uint8_t> add_header_and_bytes(const std::string& str) const;
    
        /**
         * @brief バイナリ配列を文字列に変換
         */
        std::string bytes_to_string(const std::vector<std::uint8_t>& bytes) const;
    
        TcpThread tcp_thread_;
};

#endif