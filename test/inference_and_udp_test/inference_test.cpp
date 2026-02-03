/**
 * @file test_image_processor.cpp
 * @brief ImageProcessorクラスとCameraThreadの結合テスト (UDP送信版)
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <csignal>
#include <atomic>

#include <opencv2/opencv.hpp>

#include "camera/capture_thread.hpp"
#include "image_processor/image_processor.hpp"
#include "read_resistor_value/read_resistor_value.hpp"
#include "send_image/send_image.hpp"

// 定数定義
constexpr int FRAME_WIDTH = 1280;
constexpr int FRAME_HEIGHT = 960;
constexpr char DEVICE_NAME[] = "/dev/video0";

// UDP送信設定
const std::string UDP_TARGET_HOSTNAME = "localhost"; // 受信側のホスト名を設定
constexpr std::uint16_t UDP_TARGET_PORT = 50000;

const std::string ONNX_MODEL_PATH = "models/best.onnx";
const std::string BAND_MODEL_PATH = "models/band_classifier_rf.pkl";
const std::string COLOR_MODEL_PATH = "models/resistor_color_rf_merged.pkl";

// Ctrl+C で安全に終了するためのフラグ
std::atomic<bool> g_is_running(true);

void signal_handler(int signum)
{
    g_is_running.store(false);
}

int main()
{
    // シグナルハンドラの設定
    std::signal(SIGINT, signal_handler);

    initialize_python_runtime("/home/shikoku-pc/python/venv/", "./src/inference/");

    std::cout << "Initializing ImageProcessor Test (UDP Sender Mode)..." << std::endl;

    // カメラ初期化
    V4L2Capture::frame_format current_fmt = V4L2Capture::frame_format::MJPEG;
    CaptureThread capture(
        DEVICE_NAME,
        FRAME_WIDTH,
        FRAME_HEIGHT,
        current_fmt
    );

    V4L2Capture::Frame frame_buffer;
    capture.create_empty_frame(frame_buffer);

    // 画像処理クラス初期化
    std::cout << "Loading models..." << std::endl;
    ImageProcessor processor(ONNX_MODEL_PATH, BAND_MODEL_PATH, COLOR_MODEL_PATH);

    // UDP送信クラス初期化
    std::cout << "Initializing UDP Sender..." << std::endl;
    SendImage sender(UDP_TARGET_HOSTNAME, UDP_TARGET_PORT);
    sender.create_thread();

    std::cout << "Start Capture & Processing & Sending" << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl;

    cv::Mat display_image;
    std::vector<detection_data> results;
    std::vector<std::uint8_t> jpeg_buffer;
    std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, 80}; // 画質80

    while (g_is_running.load()) {
        if (capture.get_latest_frame(frame_buffer)) {
            if (frame_buffer.size > 0) {
                // ImageProcessor::raw_image_t へのデータ詰め替え
                ImageProcessor::raw_image_t raw_img;
                
                raw_img.width = frame_buffer.width;
                raw_img.height = frame_buffer.height;
                raw_img.bytesperline = frame_buffer.bytesperline;
                raw_img.data = std::span<const uint8_t>(frame_buffer.data.data(), frame_buffer.size);

                // フォーマット変換
                if (frame_buffer.fmt == V4L2Capture::frame_format::MJPEG) {
                    raw_img.fmt = ImageProcessor::pixel_format::MJPEG;
                }
                else {
                    raw_img.fmt = ImageProcessor::pixel_format::YUV422;
                }

                // Raw -> BGR 変換
                processor.convert_raw_to_bgr(raw_img, display_image);

                results.clear();

                // 検出と描画
                bool detected = processor.detect_resistor_coordinate(display_image, results);

                if (detected) {
                    processor.draw_surround_box(display_image, results, cv::Scalar(0, 255, 0));
                }

                if (detected && raw_img.fmt == ImageProcessor::pixel_format::YUV422) {
                    std::cout << "inference resistor value" << std::endl;

                    processor.inference_resistor_value(display_image, raw_img.fmt, results);

                    for (int i = 0; i < results.size(); ++i) {
                        std::cout << "ID: " << results[i].detection_id << " ";
                        if (results[i].value.has_value()) { 
                            std::string ret = *results[i].value;
                            std::cout << "Value: " << ret << std::endl;
                        }
                        else {
                            std::cout << "Value: no value" << std::endl;
                        }
                    }
                }

                // UDP送信処理
                if (!display_image.empty()) {
                    // 推論結果(枠線など)が描画された画像をJPEGにエンコード
                    jpeg_buffer.clear();
                    cv::imencode(".jpg", display_image, jpeg_buffer, encode_params);

                    // 送信キューに追加 (std::moveで所有権移動)
                    sender.set_send_data(
                        std::move(jpeg_buffer), 
                        static_cast<std::uint16_t>(display_image.cols), 
                        static_cast<std::uint16_t>(display_image.rows), 
                        static_cast<std::uint8_t>(display_image.channels())
                    );
                }
            }
        } 
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::cout << "Stopping..." << std::endl;
    capture.stop();
    // SendImageのデストラクタでスレッド停止とjoinが行われます

    return 0;
}
