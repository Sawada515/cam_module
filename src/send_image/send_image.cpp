/**
 * @file send_image.cpp
 * @brief 画像送信クラス
 * @author sawada
 * @date 2026-02-02
 */

#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <limits>
#include <algorithm>
#include <cerrno>

#include <sys/uio.h>
#include <arpa/inet.h>

#include "logger/logger.hpp"

#include "network/udp/udp_socket.hpp"
#include "send_image/send_image.hpp"

#include "network/resolve_hostname.hpp"

constexpr int MAX_TRY_COUNT = 5;

SendImage::SendImage(std::string hostname, std::uint16_t port)
    : hostname_(std::move(hostname))
    , port_(port)
    , running_(false)
    , is_created_thread_(false)
    , buffer_state_(SendImage::buffer_state_t::EMPTY)
    , identifier_(0)
{
    while (!resolve_and_create_socket()) {
        int try_count = 0;

        spdlog::error("Failed to create socket");

        if (++try_count >= MAX_TRY_COUNT) {
            spdlog::critical("Can not resolve hostname or Failed to create socket");

            exit(EXIT_FAILURE);
        }

        usleep(1000 * 100);
    }
}

SendImage::~SendImage()
{
    {
        std::lock_guard<std::mutex> lock(mtx_);

        running_ = false;
    }

    cv_.notify_all();

    if (worker_.joinable()) {
        worker_.join();
    }
}

void SendImage::create_thread()
{
    if (is_created_thread_) {
        return;
    }

    running_.store(true);

    worker_ = std::thread(&SendImage::thread_loop, this);

    is_created_thread_ = true;
}

bool SendImage::set_send_data(std::vector<std::uint8_t>& jpeg_data, std::uint16_t width, std::uint16_t height, std::uint8_t channels)
{
    if (!is_created_thread_) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx_);

    if (buffer_state_ == SendImage::buffer_state_t::EMPTY) {
        front_jpeg_data_ = jpeg_data;

        std::memset(front_header_, 0, IMAGE_HEADER_SIZE);

        std::uint8_t* h = front_header_;

        write_header_data_info_fields(h, width, height, channels);

        buffer_state_ = buffer_state_t::FRONT_ONLY;

        cv_.notify_one();
    }
    else if (buffer_state_ == SendImage::buffer_state_t::FRONT_ONLY) {
        back_jpeg_data_ = jpeg_data;

        std::memset(back_header_, 0, IMAGE_HEADER_SIZE);

        std::uint8_t* h = back_header_;

        write_header_data_info_fields(h, width, height, channels);

        buffer_state_ = buffer_state_t::FRONT_AND_BACK;

        cv_.notify_one();
    }
    else if (buffer_state_ == SendImage::buffer_state_t::FRONT_AND_BACK) {
        back_jpeg_data_ = std::move(jpeg_data);

        std::memset(back_header_, 0, IMAGE_HEADER_SIZE);

        std::uint8_t* h = back_header_;

        write_header_data_info_fields(h, width, height, channels);
    }
    else {
        spdlog::warn("Unknown buffer type");
    }

    return true;
}

void SendImage::thread_loop()
{
    while(running_) {
        {
            std::unique_lock<std::mutex> lock(mtx_);

            cv_.wait(lock, [this] {
                return (buffer_state_ != SendImage::buffer_state_t::EMPTY) || !running_;
            });

            if (!running_) {
                break;
            }
        }

        size_t total_data_size = front_jpeg_data_.size();

        if (total_data_size > 0) {
            size_t num_packets = (total_data_size + (MAX_IMAGE_PAYLOAD_SIZE - 1)) / MAX_IMAGE_PAYLOAD_SIZE;
            
            std::uint8_t* h = front_header_;

            for (size_t i = 0; i < num_packets; ++i) {
                if (!running_) {
                    break;
                }

                size_t offset = i * MAX_IMAGE_PAYLOAD_SIZE;
                size_t chunk_size = std::min(MAX_IMAGE_PAYLOAD_SIZE, total_data_size - offset);

                write_header_data_other_info(h, identifier_, i, static_cast<std::uint16_t>(num_packets), chunk_size);

                struct iovec iov[2];
                iov[0].iov_base = front_header_;
                iov[0].iov_len = IMAGE_HEADER_SIZE;
                iov[1].iov_base = front_jpeg_data_.data() + offset;
                iov[1].iov_len = chunk_size;

                int ret = -1;

                if (socket_) {
                    ret = socket_->send_iovec(iov, 2);
                }

                if (ret < 0) {
                    spdlog::error("image send failed");

                    int err = errno;
    
                    if (err != EAGAIN && err != EWOULDBLOCK) {
                        try {
                            recreate_socket();
                        }
                        catch (const std::exception &e) {
                            spdlog::error("Failed to recreate udp socket : {}", e.what());
                        }

                        break;     //送信失敗した場合はフレームを捨てる
                    }
                }

                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        }

        if (identifier_ == UINT32_MAX) {
            identifier_ = 0;
        }
        else {
            identifier_ += 1;
        }

        {
            std::lock_guard<std::mutex> lock(mtx_);

            if (buffer_state_ == SendImage::buffer_state_t::FRONT_AND_BACK) {
                std::swap(front_jpeg_data_, back_jpeg_data_);

                std::memcpy(front_header_, back_header_, IMAGE_HEADER_SIZE);

                back_jpeg_data_.clear();

                buffer_state_ = SendImage::buffer_state_t::FRONT_ONLY;
            }
            else {
                buffer_state_ = SendImage::buffer_state_t::EMPTY;
            }
        }
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

void SendImage::write_header_data_info_fields(std::uint8_t* h, std::uint16_t width, std::uint16_t height, std::uint8_t ch)
{
    static_assert(OFFSET_WIDTH + sizeof(std::uint16_t) <= IMAGE_HEADER_SIZE);
    static_assert(OFFSET_HEIGHT + sizeof(std::uint16_t) <= IMAGE_HEADER_SIZE);
    static_assert(OFFSET_CHANNELS + sizeof(std::uint8_t) <= IMAGE_HEADER_SIZE);

    width = htons(width);
    height = htons(height);

    std::memcpy(h + OFFSET_WIDTH, &width, sizeof(width));
    std::memcpy(h + OFFSET_HEIGHT, &height, sizeof(height));
    std::memcpy(h + OFFSET_CHANNELS, &ch, sizeof(ch));
}

void SendImage::write_header_data_other_info(std::uint8_t* h, std::uint32_t identifier, std::uint16_t packet_index, std::uint16_t total_packet, std::uint16_t data_size)
{
    static_assert(OFFSET_IDENTIFIER + sizeof(std::uint32_t) <= IMAGE_HEADER_SIZE);
    static_assert(OFFSET_PACKET_INDEX + sizeof(std::uint16_t) <= IMAGE_HEADER_SIZE);
    static_assert(OFFSET_TOTAL_PACKET + sizeof(std::uint16_t) <= IMAGE_HEADER_SIZE);
    static_assert(OFFSET_DATA_SIZE + sizeof(std::uint16_t) <= IMAGE_HEADER_SIZE);

    identifier = htonl(identifier);
    packet_index = htons(packet_index);
    total_packet = htons(total_packet);
    data_size = htons(data_size);

    std::memcpy(h + OFFSET_IDENTIFIER, &identifier, sizeof(identifier));
    std::memcpy(h + OFFSET_PACKET_INDEX, &packet_index, sizeof(packet_index));
    std::memcpy(h + OFFSET_TOTAL_PACKET, &total_packet, sizeof(total_packet));
    std::memcpy(h + OFFSET_DATA_SIZE, &data_size, sizeof(data_size));
}

void SendImage::recreate_socket()
{
    {
        std::lock_guard<std::mutex> lock(mtx_);

        socket_.reset();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::unique_ptr<UdpSocket> new_socket = nullptr;

    try {
        std::string ip = resolve_mdns_ipv4(hostname_);

        new_socket = std::make_unique<UdpSocket>(ip, port_);
    }
    catch(std::exception& e) {
        spdlog::error("can not resolve hostname");

        throw;
    }

    if (new_socket) {
        std::lock_guard<std::mutex> lock(mtx_);

        socket_ = std::move(new_socket);
    }
}
