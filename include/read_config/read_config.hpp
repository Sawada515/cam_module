/**
 * @file read_config.hpp
 * @brief 設定ファイルの読み込みクラス
 * @author sawada
 * @date 2026-01-29
 */

#ifndef READ_CONFIG_HPP_
#define READ_CONFIG_HPP_

#include <string>
#include <cstdint>

#include "camera/v4l2_capture.hpp"

struct app_config_data_t {
    struct network {
        std::string dest_hostname;
        std::uint16_t tcp_json_port;
        std::uint16_t udp_send_image_port;
    } network;

    struct camera {
        std::string cam_device;
        std::uint16_t width;
        std::uint16_t height;
        V4L2Capture::frame_format default_fmt;
    } camera;

    struct image_processor {
        std::uint8_t jpeg_quality;
    } image_processor;
};

/**
 * @class ReadConfig
 * @brief YAML設定ファイル読み込みクラス
 */
class ReadConfig {
    public:
        ReadConfig();
        ~ReadConfig() = default;
    
        bool load_config(const std::string& config_file_path);
    
        const app_config_data_t& get_config_data() const;
    private:
        static V4L2Capture::frame_format parse_frame_format(const std::string& fmt);

        app_config_data_t config_data_;
};

#endif