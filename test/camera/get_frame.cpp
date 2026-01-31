/**
 * @file main.cpp
 * @brief CaptureThread 動作確認用プログラム
 * @note  OpenCVが必要です
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

#include <opencv2/opencv.hpp>

#include "camera/capture_thread.hpp"

constexpr int FRAME_WIDTH = 1280;
constexpr int FRAME_HEIGHT = 960;
constexpr char DEVICE_NAME[] = "/dev/video0";

int main()
{
    V4L2Capture::frame_format current_fmt = V4L2Capture::frame_format::MJPEG;

    std::cout << "Initializing CaptureThread..." << std::endl;

    // CaptureThreadのインスタンス化
    CaptureThread capture(
        DEVICE_NAME,
        FRAME_WIDTH,
        FRAME_HEIGHT,
        current_fmt
    );

    // データ受け取り用のフレーム構造体を準備
    V4L2Capture::Frame frame_buffer;
    
    // 【作法】内部バッファの確保
    // これを呼んでおけば、初回から適切なサイズでデータを受け取れる。
    // 万が一忘れても、v4l2_capture.cpp側で自動リサイズされるので落ちない。
    capture.create_empty_frame(frame_buffer);

    std::cout << "Start Capture: " << DEVICE_NAME << " (" << FRAME_WIDTH << "x" << FRAME_HEIGHT << ")" << std::endl;
    std::cout << "Key Commands:" << std::endl;
    std::cout << "  [c] Change Format (MJPEG <-> YUV422)" << std::endl;
    std::cout << "  [q] Quit" << std::endl;

    cv::Mat image;
    bool is_running = true;

    while (is_running) {
        // 最新フレームの取得
        // 内部でバッファ(vector)のポインタごと交換(swap)される
        if (capture.get_latest_frame(frame_buffer)) {
            
            // データサイズが有効か確認
            if (frame_buffer.size > 0) {
                // フォーマットに応じたデコード処理
                if (frame_buffer.fmt == V4L2Capture::frame_format::MJPEG) {
                    std::cout << "current format is MJPEG" << std::endl;
                    // MJPEG -> BGR
                    // データはメモリ上にあるため、imdecodeを使用（コピー発生なし）
                    cv::Mat raw_data(1, static_cast<int>(frame_buffer.size), CV_8UC1, frame_buffer.data.data());
                    image = cv::imdecode(raw_data, cv::IMREAD_COLOR);
                } 
                else if (frame_buffer.fmt == V4L2Capture::frame_format::YUV422) {
                    // YUYV -> BGR
                    // cv::Matのヘッダのみ作成（データは共有）
                    std::cout << "current format is YUV422" << std::endl;

                    cv::Mat raw(frame_buffer.height, frame_buffer.width, CV_8UC2, frame_buffer.data.data());
                    cv::cvtColor(raw, image, cv::COLOR_YUV2BGR_YUYV);
                }

                // 表示
                if (!image.empty()) {
                    cv::imshow("Capture Test", image);
                }
            }
        } else {
            // データが来ていない場合はCPU負荷を下げるため少し待機
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // キー入力待ち (1ms)
        int key = cv::waitKey(1);
        
        if (key == 'q') {
            is_running = false;
        } 
        else if (key == 'c') {
            // フォーマットの切り替えトグル
            if (current_fmt == V4L2Capture::frame_format::MJPEG) {
                current_fmt = V4L2Capture::frame_format::YUV422;
                std::cout << "Request Change Format -> YUV422" << std::endl;
            } else {
                current_fmt = V4L2Capture::frame_format::MJPEG;
                std::cout << "Request Change Format -> MJPEG" << std::endl;
            }
            
            // 変更リクエスト
            // CaptureThread内で非同期に処理される
            capture.request_change_format(current_fmt);
        }
    }

    // 終了処理
    std::cout << "Stopping..." << std::endl;
    capture.stop();
    cv::destroyAllWindows();

    return 0;
}
