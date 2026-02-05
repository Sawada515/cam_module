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
#include <climits>
#include <exception>
#include <atomic>
#include <csignal>
#include <optional>
#include <span>

#include <opencv2/core/mat.hpp>

#include "camera/capture_thread.hpp"
#include "image_processor/image_processor.hpp"
#include "json/json_client.hpp"
#include "logger/logger.hpp"
#include "read_config/read_config.hpp"
#include "read_resistor_value/read_resistor_value.hpp"
#include "send_image/send_image.hpp"

#include <iostream>

std::atomic<bool> g_is_running(true);

constexpr const char* CONFIG_FILE = "./config/config.yaml";

void signal_handler([[maybe_unused]] int signum);

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

    std::cout << "finish init\n";

    json_client.start();
    send_image.create_thread();

    std::vector<std::uint8_t> jpeg_buffer;

    const size_t BGR_SIZE = config.camera.width * config.camera.height * 3;
    const size_t ESTIMATE_JPEG_SIZE = BGR_SIZE / 2; //BGRのサイズの50%をjpeg_bufferのサイズとして確保しておく

    jpeg_buffer.reserve(ESTIMATE_JPEG_SIZE);

    std::vector<detection_data> detect_results;

    cv::Mat bgr_mat;

    bool is_inference_and_send_json = false;
    int send_json_frame_id = 0;

    std::cout << "start\n";

    while(g_is_running.load()) {
        std::cerr << "1\n";
        std::optional<JsonClient::recv_cmd_data> recv_json_data = json_client.try_receive();
        if (recv_json_data.has_value()) {
            JsonClient::recv_cmd_data recv_data = recv_json_data.value();

            if (recv_data.cmd == JsonClient::cmd_kinds::SHUTDOWN) {
                spdlog::info("recv json cmd shutdown");

                //std::system("shutdown -h now");
                break;
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
            else {
                spdlog::warn("Unknown command");
            }
        }
        std::cerr << "2\n";

        if(!camera.get_latest_frame(frame_buffer)) {
            continue;

            std::cerr << "get frame err\n";

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::cerr << "3\n";

        if (frame_buffer.size > 0) {
            spdlog::info("get frame OK");

            ImageProcessor::raw_image_t  raw_image{
                frame_buffer.width,
                frame_buffer.height,
                (frame_buffer.fmt == V4L2Capture::frame_format::MJPEG) ? ImageProcessor::pixel_format::MJPEG : ImageProcessor::pixel_format::YUV422,
                frame_buffer.bytesperline,
                std::span<const uint8_t>(frame_buffer.data.data(), frame_buffer.size)
            };

            image_processor.convert_raw_to_bgr(raw_image, bgr_mat);

            detect_results.clear();

            bool is_detected = image_processor.detect_resistor_coordinate(bgr_mat, detect_results);

            if (is_detected && raw_image.fmt == ImageProcessor::pixel_format::MJPEG) {
                image_processor.draw_surround_box(bgr_mat, detect_results, cv::Scalar(0, 0, 255));

                is_inference_and_send_json = false;
            }
            else if (is_detected && raw_image.fmt == ImageProcessor::pixel_format::YUV422 && !is_inference_and_send_json) {
                image_processor.inference_resistor_value(bgr_mat, raw_image.fmt, detect_results);

                if (send_json_frame_id == INT_MAX) {
                    send_json_frame_id = 0;
                }

                type_data_json send_data{
                    "data",
                    send_json_frame_id++,
                    detect_results
                };

                spdlog::error("data send?");

                json_client.send_data(send_data);

                camera.request_change_format(V4L2Capture::frame_format::MJPEG);

                is_inference_and_send_json = true; 
            }

            jpeg_buffer.clear();

            if(image_processor.jpeg_compression_bgr_data(bgr_mat, jpeg_buffer, config.image_processor.jpeg_quality)) {
                spdlog::info("send image data ok");
                send_image.set_send_data(
                    jpeg_buffer,
                    static_cast<std::uint16_t>(bgr_mat.cols),
                    static_cast<std::uint16_t>(bgr_mat.rows),
                    static_cast<std::uint8_t>(bgr_mat.channels())
                );
            }
            else {
                spdlog::error("Failed to jpeg compression");
            }
        }
    }

    return 0;
}

void signal_handler([[maybe_unused]] int signum)
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
