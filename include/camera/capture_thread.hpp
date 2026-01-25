/**
 * @file capture_thread.hpp
 * @brief 画像データをスレッドで取得
 * @author sawada
 * @date 2026-01-25
 */

#ifndef CAPTURE_THREAD_HPP_
#define CAPTURE_THREAD_HPP_

#include <thread>
#include <mutex>
#include <atomic>
#include <optional>
#include <cstdint>

#include "logger/logger.hpp"
#include "camera/v4l2_capture.hpp"

constexpr std::uint16_t YUV422_BYTES_PER_PIXEL = 2;

/**
 * @class CaptureThread
 * @brief 画像データをスレッドで取得するためのクラス 
 */
class CaptureThread {
    public:
        CaptureThread(
            const std::string& device_file_name,
            std::uint16_t width,
            std::uint16_t height,
            V4L2Capture::frame_format fmt
        );

        ~CaptureThread();

        CaptureThread(const CaptureThread&) = delete;
        CaptureThread& operator=(const CaptureThread&) = delete;

        /**
         * @brief V4L2Capture::Frameのvector dataの領域を確保
         * @note 呼び出しもとでFrameデータを作成したら1回必ず呼び出し
         */
        void create_empty_frame(V4L2Capture::Frame& frame);

        /**
         * @brief 最新フレームを返す
         * @return true 最新データあり
         * @return false データなし
         */
        bool get_latest_frame(V4L2Capture::Frame& frame);

        /**
         * @brief フォーマットを変更する必要がある場合に呼び出し
         */
        void request_change_format(V4L2Capture::frame_format fmt);
    
        /**
         * スレッドを停止する必要がある場合に呼び出し
         */
        void stop();
    private:
        const std::string device_file_name_;

        const std::uint16_t width_;
        const std::uint16_t height_;

        const size_t require_buffer_size_;

        std::optional<V4L2Capture::Frame> latest_frame_;

        mutable std::mutex frame_mtx_;

        std::thread worker_;
        std::atomic<bool> stop_flag_ = {false};

        bool has_new_frame_ = false;

        std::optional<V4L2Capture::frame_format> request_fmt_;
        V4L2Capture::frame_format current_fmt_;

        std::mutex fmt_mtx_;

        void thread_loop();
};

#endif 
