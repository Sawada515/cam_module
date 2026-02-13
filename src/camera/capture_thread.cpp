/**
 * @file capture_thread.cpp
 * @brief 画像データをスレッドで取得
 * @author sawada
 * @date 2026-01-25
 */

#include <utility>
#include <chrono>
#include <system_error>

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
    create_empty_frame(latest_frame_);

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

    frame.valid_size = 0;

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

    if (!has_new_frame_ || latest_frame_.valid_size == 0) {
        return false;
    }

    std::swap(frame, latest_frame_);

    has_new_frame_ = false;

    return true;
}

void CaptureThread::stop()
{
    stop_flag_.store(true);
}

void CaptureThread::thread_loop()
{
    std::optional<V4L2Capture> camera;

    try {
        camera.emplace(
            device_file_name_,
            width_,
            height_,
            current_fmt_
        );

        V4L2Capture::Frame back_frame;

        create_empty_frame(back_frame);

        try {
            camera->stream_on();
        }
        catch (const std::system_error& e) {
            spdlog::error("camera error: {} (code: {})", e.what(), e.code().value());

            return;
        }

        while(!stop_flag_.load(std::memory_order_relaxed)) {
            {
                std::unique_lock<std::mutex> lock(fmt_mtx_);
                if (request_fmt_.has_value()) {
                    change_format(camera, request_fmt_.value());
                }

                request_fmt_ = std::nullopt;
            }

            if (!camera.has_value()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                continue;
            }

            try {
                if(!camera->capture_frame(back_frame)) {
                    continue;
                }
            }
            catch (const std::exception& e) {
                spdlog::warn("dtop mjepg data");

                continue;
            }

            {
                std::lock_guard<std::mutex> lock(frame_mtx_);

                std::swap(latest_frame_, back_frame);

                has_new_frame_ = true;
            }
        }
    }
    catch (const std::exception& e) {
        spdlog::error("Capture thread error: {}", e.what());
    }

    spdlog::info("Capture thread stopped");
}

void CaptureThread::change_format(std::optional<V4L2Capture>& camera, V4L2Capture::frame_format reqest_fmt)
{
    try {
        if (camera.has_value()) {
            camera->reconfigure(reqest_fmt);
        }
        else {
            camera.emplace(device_file_name_, width_, height_, reqest_fmt);
        }

        current_fmt_ = reqest_fmt;
    }
    catch (const std::exception& e) {
        spdlog::error("Reconfigure failed: {}", e.what());

        camera.reset();

        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            camera.emplace(device_file_name_, width_, height_, reqest_fmt);
            camera->stream_on();

            current_fmt_ = reqest_fmt;

            spdlog::info("Camera recovered");
        }
        catch (const std::exception& e) {
            spdlog::critical("Recovery failed: {}", e.what());
        }
    }
}