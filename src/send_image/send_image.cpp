/**
 * @file send_image.cpp
 * @brief 画像送信クラス
 * @author sawada
 * @date 2026-02-02
 */

#include <cstring>
#include <stdexcept>
#include <limits>

#include <sys/uio.h>
#include <arpa/inet.h>

#include "logger/logger.hpp"

#include "network/udp/udp_socket.hpp"
#include "send_image/send_image.hpp"

#include "network/resolve_hostname.hpp"


SendImage::SendImage(std::string hostname, std::uint16_t port)
    : hostname_(std::move(hostname))
    , port_(port)
    , running_(false)
    , is_created_thread_(false)
    , identifier_(0)
    , packet_index_(0)
    , total_packet_(0)
{
    if (!SendImage::resolve_and_create_socket()) {
        throw std::runtime_error("Failed to create socket");
    }
}

bool SendImage::resolve_and_create_socket()
{
    try {
        std::string ip = resolve_mdns_ipv4(hostname_);

        socket_ = std::make_unique<UdpSocket>(ip, port_);

        return true;
    }
    catch (const std::exception& e) {
        spdlog::warn("UDP resolve/connect failed: {}", e.what());

        socket_.reset();

        return false;
    }
}

