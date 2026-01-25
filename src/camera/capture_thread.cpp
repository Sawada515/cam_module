/**
 * @file capture_thread.cpp
 * @brief 画像データをスレッドで取得
 * @author sawada
 * @date 2026-01-25
 */

#include <utility>
#include <chrono>

#include "logger/logger.hpp"
#include "camera/v4l2_capture.hpp"
#include "camera/capture_thread.hpp"

CaptureThread::CaptureThread(
    const std::string& device_file_name,
    std::uint16_t width,
    std::uint16_t height,
    V4L2Capture::frame_format fmt
)
    : device_file_name_(device_file_name)
    , width_(width)
    , height_(height)
    , require_buffer_size_(width * height * YUV422_BYTES_PER_PIXEL)
    , current_fmt_(fmt)
{
    worker_ = std::thread(&CaptureThread::thread_loop, this);

    spdlog::info("CaptureThread started: {}, {}x{}", device_file_name_, width_, height_);
}

CaptureThread::~CaptureThread()
{
    stop();

    if (worker_.joinable()) {
        worker_.join();
    }
}

void CaptureThread::create_empty_frame(V4L2Capture::Frame& frame)
{
    frame.width = width_;
    frame.height = height_;
    frame.fmt = current_fmt_;

    frame.size = 0;

    if (frame.data.size() < require_buffer_size_) {
        frame.data.resize(require_buffer_size_);
    }
}

void CaptureThread::request_change_format(V4L2Capture::frame_format fmt)
{
    std::lock_guard<std::mutex> lock(fmt_mtx_);

    request_fmt_ = fmt;
}

bool CaptureThread::get_latest_frame(V4L2Capture::Frame& frame)
{
    std::lock_guard<std::mutex> lock(frame_mtx_);

    if (!has_new_frame_ || !latest_frame_.has_value()) {
        return false;
    }

    std::swap(frame, *latest_frame_);

    has_new_frame_ = false;

    return true;
}

void CaptureThread::stop()
{
    stop_flag_.store(true);
}