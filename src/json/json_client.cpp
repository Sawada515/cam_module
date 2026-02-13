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

namespace {
    struct CmdConverter
    {
        using ResultType = std::optional<JsonClient::recv_cmd_data>;
    
        ResultType operator()(const type_cmd_json& cmd_json) const
        {
            JsonClient::recv_cmd_data result = { JsonClient::cmd_kinds::UNKNOWN, JsonClient::args_kinds::NONE };
    
            if (cmd_json.command == "change_format") {
                result.cmd = JsonClient::cmd_kinds::CHANGE_FORMAT;
    
                if (std::holds_alternative<Change_format_args>(cmd_json.args)) {
                    auto args = std::get<Change_format_args>(cmd_json.args);
                    
                    if (args.format == "MJPEG") {
                        result.args = JsonClient::args_kinds::MJPEG;
                    } else if (args.format == "YUV422") {
                        result.args = JsonClient::args_kinds::YUV422;
                    }
                }
                return result;
            } else if (cmd_json.command == "shutdown") {
                result.cmd = JsonClient::cmd_kinds::SHUTDOWN;

                return result;
            }
    
            return std::nullopt;
        }
    
        ResultType operator()(const type_data_json&) const
        {
            return std::nullopt;
        }
    };
}

namespace {
    constexpr size_t RECV_BUFFER_DEFAULT_SIZE = 2048;

    constexpr size_t HEADER_SIZE = 4;
}

JsonClient::JsonClient(const std::string& hostname, std::uint16_t port)
    : tcp_thread_(hostname, port)
{
    recv_buffer_.resize(RECV_BUFFER_DEFAULT_SIZE);
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

std::optional<JsonClient::recv_cmd_data> JsonClient::try_receive()
{
    tcp_thread_.fetch_recv_data(recv_buffer_);

    if (recv_buffer_.empty()) {
        return std::nullopt;
    }
    

    if (require_recv_packet_size_ == 0) {
        if (recv_buffer_.size() < HEADER_SIZE) {
            return std::nullopt;
        }

        std::uint32_t network_order_byte_length = 0;
        std::memcpy(&network_order_byte_length, recv_buffer_.data(), HEADER_SIZE);

        std::uint32_t body_length = ntohl(network_order_byte_length);

        require_recv_packet_size_ = body_length + HEADER_SIZE;

        if (require_recv_packet_size_ == 0) {
            spdlog::warn("recv data is None");

            recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + require_recv_packet_size_);

            return std::nullopt;
        }
    }

    if (recv_buffer_.size() < require_recv_packet_size_) {
        return std::nullopt;
    }

    std::string json_str(recv_buffer_.begin() + HEADER_SIZE, recv_buffer_.begin() + require_recv_packet_size_);

    recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + require_recv_packet_size_);

    require_recv_packet_size_ = 0;

    return json_str_to_recv_cmd_data(json_str);
}

std::vector<std::uint8_t> JsonClient::add_header_and_bytes(const std::string& str) const
{
    std::uint32_t body_len = static_cast<std::uint32_t>(str.size());

    std::vector<std::uint8_t> buffer;
    buffer.reserve(HEADER_SIZE + body_len);

    std::uint32_t net_len = htonl(body_len);
    const std::uint8_t* len_ptr = reinterpret_cast<const std::uint8_t*>(&net_len);

    buffer.insert(buffer.end(), len_ptr, len_ptr + HEADER_SIZE);

    buffer.insert(buffer.end(), str.begin(), str.end());

    return buffer;
}

std::optional<JsonClient::recv_cmd_data> JsonClient::json_str_to_recv_cmd_data(std::string json_str)
{
    try {
        auto packet_variant = JsonCodec::parse(json_str);

        return std::visit(CmdConverter{}, packet_variant);
    }
    catch (const std::exception& e) {
        spdlog::error("JsonClient: Parse or Convert Error: {}", e.what());

        return std::nullopt;
    }
}