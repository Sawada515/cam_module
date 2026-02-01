/**
 * @file test_image_processor.cpp
 * @brief ImageProcessorクラスとCameraThreadの結合テスト
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "camera/capture_thread.hpp"
#include "image_processor/image_processor.hpp"
#include "read_resistor_value/read_resistor_value.hpp"

// 定数定義
constexpr int FRAME_WIDTH = 1280;
constexpr int FRAME_HEIGHT = 960;
constexpr char DEVICE_NAME[] = "/dev/video0";

const std::string ONNX_MODEL_PATH = "models/best.onnx";
const std::string BAND_MODEL_PATH = "models/band_classifier_rf.pkl";
const std::string COLOR_MODEL_PATH = "models/resistor_color_rf_merged.pkl";

int main()
{
    initialize_python_runtime("/home/sawada/python/venv/", "./src/inference/");

    std::cout << "Initializing ImageProcessor Test..." << std::endl;

    V4L2Capture::frame_format current_fmt = V4L2Capture::frame_format::MJPEG;
    CaptureThread capture(
        DEVICE_NAME,
        FRAME_WIDTH,
        FRAME_HEIGHT,
        current_fmt
    );

    V4L2Capture::Frame frame_buffer;
    capture.create_empty_frame(frame_buffer);

    std::cout << "Loading models..." << std::endl;
    ImageProcessor processor(ONNX_MODEL_PATH, BAND_MODEL_PATH, COLOR_MODEL_PATH);

    std::cout << "Start Capture & Processing" << std::endl;
    std::cout << "Key Commands:" << std::endl;
    std::cout << "  [c] Change Format (MJPEG <-> YUV422)" << std::endl;
    std::cout << "  [q] Quit" << std::endl;

    bool is_running = true;
    cv::Mat display_image;
    std::vector<detection_data> results;

    while (is_running) {
        if (capture.get_latest_frame(frame_buffer)) {
            if (frame_buffer.size > 0) {
                // ImageProcessor::raw_image_t へのデータ詰め替え
                ImageProcessor::raw_image_t raw_img;
                
                raw_img.width = frame_buffer.width;
                raw_img.height = frame_buffer.height;
                raw_img.bytesperline = frame_buffer.bytesperline; // 提供された値をそのまま使用
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
                        std::cout << results[i].detection_id << std::endl;
                        if (results[i].value.has_value()) { 
                            std::string ret = *results[i].value;
                            std::cout << ret << std::endl;
                        }
                        else {
                            std::cout << "no value" << std::endl;
                        }
                    }
                }

                if (!display_image.empty()) {
                    cv::imshow("ImageProcessor Test", display_image);
                }
            }
        } 
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        int key = cv::waitKey(1);
        if (key == 'q') {
            is_running = false;
        } 
        else if (key == 'c') {
            if (current_fmt == V4L2Capture::frame_format::MJPEG) {
                current_fmt = V4L2Capture::frame_format::YUV422;
                std::cout << "Request Change Format -> YUV422" << std::endl;
            } 
            else {
                current_fmt = V4L2Capture::frame_format::MJPEG;
                std::cout << "Request Change Format -> MJPEG" << std::endl;
            }
            capture.request_change_format(current_fmt);
        }
    }

    std::cout << "Stopping..." << std::endl;
    capture.stop();
    cv::destroyAllWindows();

    return 0;
}
