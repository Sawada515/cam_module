/**
 * @file json_client.cpp
 * @brief JsonClient implementation
 * @author sawada
 * @date 2026-01-30
 */

#include <spdlog/spdlog.h>
#include <arpa/inet.h>
#include <vector>
#include <cstring>
#include <stdexcept>

#include "json/json_client.hpp"
#include "logger/logger.hpp"

JsonClient::JsonClient(const std::string& hostname, std::uint16_t port)
    : tcp_thread_(hostname, port)
{
}

JsonClient::~JsonClient()
{
    stop();
}

void JsonClient::start()
{
    spdlog::info("JsonClient: Starting TCP thread...");

    tcp_thread_.start();
}

void JsonClient::stop()
{
    spdlog::info("JsonClient: Stopping TCP thread...");

    tcp_thread_.stop();
}

void JsonClient::send_command(const type_cmd_json& cmd)
{
    try {
        std::string jsonStr = JsonCodec::serialize_cmd(cmd);

        std::vector<std::uint8_t> packet = add_header_and_bytes(jsonStr);

        tcp_thread_.send(std::move(packet));
    }
    catch (const std::exception& e) {
        spdlog::error("JsonClient: Failed to send command. Error: {}", e.what());
    }
}

void JsonClient::send_data(const type_data_json& data)
{
    try {
        std::string jsonStr = JsonCodec::serialize_data(data);
        std::vector<std::uint8_t> packet = add_header_and_bytes(jsonStr);

        tcp_thread_.send(std::move(packet));
    }
    catch (const std::exception& e) {
        spdlog::error("JsonClient: Failed to send data. Error: {}", e.what());
    }
}

std::optional<JsonCodec::PacketVariant> JsonClient::try_receive()
{
    if (!tcp_thread_.has_received_data()) {
        return std::nullopt;
    }

    std::vector<std::uint8_t> recvBytes;
    if (tcp_thread_.recv(recvBytes)) {
        if (recvBytes.empty()) {
            return std::nullopt;
        }

        std::string jsonStr = bytes_to_string(recvBytes);

        try {
            return JsonCodec::parse(jsonStr);
        }
        catch (const std::exception& e) {
            spdlog::error("JsonClient: Parse Error: {}", e.what());

            return std::nullopt;
        }
    }

    return std::nullopt;
}

std::vector<std::uint8_t> JsonClient::add_header_and_bytes(const std::string& str) const
{
    constexpr size_t HEADER_SIZE = 4;
    std::uint32_t body_len = static_cast<std::uint32_t>(str.size());

    std::vector<std::uint8_t> buffer;
    buffer.reserve(HEADER_SIZE + body_len);

    std::uint32_t net_len = htonl(body_len);
    const std::uint8_t* len_ptr = reinterpret_cast<const std::uint8_t*>(&net_len);

    buffer.insert(buffer.end(), len_ptr, len_ptr + HEADER_SIZE);

    buffer.insert(buffer.end(), str.begin(), str.end());

    return buffer;
}

std::string JsonClient::bytes_to_string(const std::vector<std::uint8_t>& bytes) const
{
    return std::string(bytes.begin(), bytes.end());
}
