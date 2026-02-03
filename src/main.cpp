/**
 * @file main.cpp
 * @brief main関数
 * @author sawada
 * @date 2026-02-04
 */

#include <string>
#include <cstdlib>
#include <cstdint>
#include <vector>
#include <exception>
#include <atomic>
#include <csignal>
#include <optional>
#include <span>

#include "camera/capture_thread.hpp"
#include "image_processor/image_processor.hpp"
#include "json/json_client.hpp"
#include "logger/logger.hpp"
#include "read_config/read_config.hpp"
#include "send_image/send_image.hpp"

std::atomic<bool> g_is_running(true);

constexpr const char* CONFIG_FILE = "./config/config.yaml";

app_config_data_t read_config(const std::string &config_file);

int main()
{
    std::signal(SIGINT, signal_handler);

    app_config_data_t config = read_config(CONFIG_FILE);

    initialize_python_runtime(config.python.venv_path, config.python.script_path);

    ImageProcessor image_processor(config.image_processor.onnx_model_path, config.image_processor.band_model_path, config.image_processor.color_model_path);

    ImageProcessor::raw_image_t raw_image_buffer;

    JsonClient json_client(config.network.dest_hostname, config.network.tcp_json_port);
    
    SendImage send_image(config.network.dest_hostname, config.network.udp_send_image_port);

    CaptureThread camera(config.camera.cam_device, config.camera.width, config.camera.height, config.camera.default_fmt);

    V4L2Capture::Frame frame_buffer;
    camera.create_empty_frame(frame_buffer);

    json_client.start();
    send_image.create_thread();

    while(g_is_running.load()) {
        std::optional<JsonClient::recv_cmd_data> recv_json_data = json_client.try_receive();
        if (recv_json_data.has_value()) {
            JsonClient::recv_cmd_data recv_data = recv_json_data.value();

            if (recv_data.cmd == JsonClient::cmd_kinds::SHUTDOWN) {
                spdlog::info("recv json cmd shutdown");

                std::system("shutdown -h now");
            }
            else if (recv_data.cmd == JsonClient::cmd_kinds::CHANGE_FORMAT) {
                if (recv_data.args == JsonClient::args_kinds::MJPEG) {
                    camera.request_change_format(V4L2Capture::frame_format::MJPEG);

                    spdlog::info("change format MJPEG");
                }
                else if (recv_data.args == JsonClient::args_kinds::YUV422) {
                    camera.request_change_format(V4L2Capture::frame_format::YUV422);

                    spdlog::info("change format YUV422");
                }
                else {
                    spdlog::warn("Unknown change format args type");
                }
            }
        }
    }
}

void signal_handler(int signum __attribute__((unused)))
{
    g_is_running.store(false);
}

app_config_data_t read_config(const std::string &config_file)
{
    ReadConfig read_config;

    bool ret = false;

    try {
        ret = read_config.load_config(config_file);
    }
    catch (const std::exception& e) {
        spdlog::critical("Exception during config loading: {}", e.what());

        exit(EXIT_FAILURE);
    }

    if (!ret) {
        spdlog::error("Failed to load {}", config_file);

        exit(EXIT_FAILURE);
    }

    return read_config.get_config_data();
}

