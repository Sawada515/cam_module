/**
 * @file read_config.cpp
 * @brief 設定ファイル読み込みクラス実装
 * @author sawada
 * @date 2026-01-29
 * @todo タイポ対策でキーを定数にする
 */

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

#include "read_config/read_config.hpp"

#include "logger/logger.hpp"

ReadConfig::ReadConfig()
{
    config_data_.network.dest_hostname = "localhost";
    config_data_.network.tcp_json_port = 55555;
    config_data_.network.udp_send_image_port = 50000;

    config_data_.camera.cam_device = "/dev/video0";
    config_data_.camera.width = 1280;
    config_data_.camera.height = 960;
    config_data_.camera.default_fmt = V4L2Capture::frame_format::MJPEG;

    config_data_.image_processor.jpeg_quality = 80;

    config_data_.python.venv_path = "/home/sawada/python/venv/";
    config_data_.python.script_path = "./src/inference";
}

bool ReadConfig::load_config(const std::string& config_file_path)
{
    try {
        const YAML::Node config = YAML::LoadFile(config_file_path);

        if (!config["network"]) {
            throw std::runtime_error("network section missing");
        }
        {
            const auto net = config["network"];

            if (
                !net["dest_hostname"] || !net["tcp_json_port"] || !net["udp_send_image_port"]
            ) {
                throw std::runtime_error("network section missing required keys");
            }
            
            config_data_.network.dest_hostname = net["dest_hostname"].as<std::string>();
            config_data_.network.tcp_json_port = net["tcp_json_port"].as<std::uint16_t>();
            config_data_.network.udp_send_image_port = net["udp_send_image_port"].as<std::uint16_t>();
        }

        if (!config["camera"]) {
            throw std::runtime_error("camera section missing");
        }
        {
            const auto cam = config["camera"];

            if (
                !cam["cam_device"] || !cam["width"] || !cam["height"] || !cam["default_fmt"]
            ) {
                throw std::runtime_error("camera section missing required keys");
            }

            config_data_.camera.cam_device = cam["cam_device"].as<std::string>();
            config_data_.camera.width = cam["width"].as<std::uint16_t>();
            config_data_.camera.height = cam["height"].as<std::uint16_t>();

            config_data_.camera.default_fmt = parse_frame_format(cam["default_fmt"].as<std::string>());
        }

        if (!config["image_processor"]) {
            throw std::runtime_error("image_processor section missing");
        }
        {
            const auto img = config["image_processor"];

            if (!img["jpeg_quality"]) {
                throw std::runtime_error("image_processor section missing required keys");
            }

            const int quality = img["jpeg_quality"].as<int>();

            if (quality < 0 || quality > 100) {
                throw std::runtime_error("jpeg_quality out of range(0-100)");
            }

            config_data_.image_processor.jpeg_quality = static_cast<std::uint8_t>(quality);
        }

        if (!config["python"]) {
            throw std::runtime_error("python section missing");
        }
        {
            const auto py = config["python"];

            if (!py["venv_path"] || !py["script_path"]) {
                throw std::runtime_error("venv_path section missing required keys");
            }

            config_data_.python.venv_path = py["venv_path"].as<std::string>();

            config_data_.python.script_path = py["script_path"].as<std::string>();
        }
    }
    catch (const YAML::BadFile& e) {
        spdlog::error("Failed to open config file: {}", e.what());

        return false;
    }
    catch (const YAML::Exception& e) {
        spdlog::error("YAML parse error: {}", e.what());

        return false;
    }
    catch (const std::exception& e) {
        spdlog::error("Config error: {}", e.what());

        return false;
    }

    return true;
}

const app_config_data_t& ReadConfig::get_config_data() const
{
    return config_data_;
}

V4L2Capture::frame_format ReadConfig::parse_frame_format(const std::string& fmt)
{
    if (fmt == "MJPEG") {
        return V4L2Capture::frame_format::MJPEG;
    }
    if (fmt == "YUV422") {
        return V4L2Capture::frame_format::YUV422;
    }

    throw std::runtime_error("Invalid camera default format: " + fmt + " (expected MJPEG or YUV422)");
}
