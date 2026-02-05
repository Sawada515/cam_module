/**
 * @file v4l2_capture.cpp
 * @brief V4L2ドライバを使ってカメラデバイスの操作
 * @author sawada
 * @date 2026-01-24
 */

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <poll.h>
#include <linux/videodev2.h>

#include <cstdint>
#include <string>
#include <cstring>
#include <cerrno>
#include <system_error>

#include "camera/v4l2_capture.hpp"
#include "logger/logger.hpp"

static int xioctl(int fd, unsigned long req, void* arg);
static int xpoll(struct pollfd* poll_fds, nfds_t nfds, int timeout);

static int xioctl(int fd, unsigned long req, void* arg)
{
    int ret;

    do {
        ret = ioctl(fd, req, arg);
    } while(ret == -1 && errno == EINTR);

    return ret;
}

static int xpoll(struct pollfd* poll_fds, nfds_t nfds, int timeout)
{
    int ret;

    do {
        ret = poll(poll_fds, nfds, timeout);
    } while (ret == -1 && errno == EINTR);

    return ret;
}

V4L2Capture::V4L2Capture(std::string device_file_name, std::uint16_t width, std::uint16_t height, frame_format fmt)
    : device_file_name_(std::move(device_file_name))
    , width_(width)
    , height_(height)
    , fmt_(fmt)
    , device_fd_(open_device())
{
    try {
        set_frame_format();
    }
    catch(const std::system_error& e)
    {
        throw;
    }

    spdlog::info("initialized: {} ({}x{})", device_file_name_, width_, height_);

    try {
        request_capture_buffer();
    }
    catch (const std::system_error& e)
    {
        cleanup_buffers();

        throw;
    }
}

V4L2Capture::~V4L2Capture()
{
    try {
        stream_off();
    }
    catch(const std::system_error& e)
    {
        ;   //無視
    }

    cleanup_buffers();    

    close_device();
}


bool V4L2Capture::capture_frame(V4L2Capture::Frame& frame)
{
    pollfd poll_fd{};

    poll_fd.fd = device_fd_;
    poll_fd.events = POLLIN;

    int ret = xpoll(&poll_fd, 1, 1000);

    if (ret == 0) {
        frame.size = 0;

        return false;
    }
    else if (ret < 0) {
        throw std::system_error(errno, std::generic_category(), "poll failed");
    }

    v4l2_buffer buf{};

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(device_fd_, VIDIOC_DQBUF, &buf) < 0) {
        throw std::system_error(errno, std::generic_category(), "Failed to dequeue buffer");
    }

    if (buf.index >= buffers_.size()) {
        throw std::out_of_range("Buffer index out of bounds from driver");
    }

    const std::uint8_t* start_ptr = static_cast<const std::uint8_t*>(buffers_[buf.index].start);
    
    size_t data_size = buf.bytesused;

    if (data_size > frame.data.size()) {
        frame.data.resize(data_size);
    }

    std::memcpy(frame.data.data(), start_ptr, data_size);

    frame.width = width_;
    frame.height = height_;

    frame.size = data_size;

    frame.bytesperline = bytesperline_;

    frame.fmt = fmt_;

    if (xioctl(device_fd_, VIDIOC_QBUF, &buf) < 0) {
        throw std::system_error(errno, std::generic_category(), "VIDIOC_QBUF failed");
    }

    return true;
}

void V4L2Capture::stream_on()
{
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(device_fd_, VIDIOC_STREAMON, &type) < 0) {
        throw std::system_error(errno, std::generic_category(), "VIDIOC_STREAMON failed");
    }
}

void V4L2Capture::stream_off()
{
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(device_fd_, VIDIOC_STREAMOFF, &type) < 0) {
        throw std::system_error(errno, std::generic_category(), "Failed to stream off");
    }
}

void V4L2Capture::reconfigure(frame_format fmt)
{
    constexpr int MAX_RETRY = 5;
    constexpr int DELAY_US = 5000;

    bool is_pass_fmt_check;

    //usleep(1000 * 10);

    for (int i = 0; i < MAX_RETRY; ++i) {
        is_pass_fmt_check = try_format(width_, height_, static_cast<std::uint32_t>(fmt));

        if(!is_pass_fmt_check && (errno == EBUSY || errno == EAGAIN)) {
            usleep(DELAY_US);
        }
        else {
            break;
        }
    }

    if(!is_pass_fmt_check) {
        throw std::runtime_error("not supprted fmt");
    }

    stream_off();

    cleanup_buffers();

    fmt_ = fmt;

    set_frame_format();

    request_capture_buffer();

    stream_on();

    constexpr int DROP_FRAM_NUM = 30;

    drop_frame(DROP_FRAM_NUM);
}

int V4L2Capture::open_device()
{
    int device_fd = ::open(device_file_name_.c_str(), O_RDWR | O_NONBLOCK);

    if (device_fd < 0) {
        throw std::system_error(errno, std::generic_category(), "Failed to open device: " + device_file_name_);
    }

    return device_fd;
}

void V4L2Capture::close_device()
{
    if (device_fd_ < 0) {
        return;
    }

    ::close(device_fd_);

    device_fd_ = -1;
}

void V4L2Capture::request_capture_buffer()
{
    v4l2_requestbuffers req{};
    
    req.count = CAPTURE_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(device_fd_, VIDIOC_REQBUFS, &req) < 0) {
        throw std::system_error(errno, std::generic_category(), "VIDEOC_QUERYBUF failed");
    }

    buffers_.resize(req.count);

    for (size_t i = 0; i < buffers_.size(); ++i) {
        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;

        if (xioctl(device_fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            throw std::system_error(errno, std::generic_category(), "VIDEOC_QUERYBUF failed");
        }

        buffers_[i].length = buf.length;
        buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, device_fd_, buf.m.offset);

        if (buffers_[i].start == MAP_FAILED) {
            throw std::system_error(errno, std::generic_category(), "MMAP failed");
        }

        if (xioctl(device_fd_, VIDIOC_QBUF, &buf) < 0) {
            throw std::system_error(errno, std::generic_category(), "VIDIOC_QBUF failed");
        }
    }
}

void V4L2Capture::set_frame_format()
{
    v4l2_format fmt{};

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;

    fmt.fmt.pix.pixelformat = static_cast<std::uint32_t>(fmt_);

    //fmt.fmt.pix.field = V4L2_FIELD_NONE;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (xioctl(device_fd_, VIDIOC_S_FMT, &fmt) < 0) {
        throw std::system_error(errno, std::generic_category(), "invalid format");
    }

    if (width_ != fmt.fmt.pix.width) {
        width_ = fmt.fmt.pix.width;
    }
    if (height_ != fmt.fmt.pix.height) {
        height_ = fmt.fmt.pix.height;
    }

    spdlog::info("set format {}x{}", width_, height_);

    bytesperline_ = fmt.fmt.pix.bytesperline;
}

void V4L2Capture::cleanup_buffers()
{
    for (auto& buf : buffers_) {
        if (buf.start && buf.start != MAP_FAILED) {
            munmap(buf.start, buf.length);
        }
    }

    buffers_.clear();

    v4l2_requestbuffers req{};
    req.count = 0;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(device_fd_, VIDIOC_REQBUFS, &req) < 0) {
        spdlog::warn("cleanup_buffers: REQBUFS(0) failed. errno={}", errno);
    }
}

bool V4L2Capture::try_format(std::uint16_t width, std::uint16_t height, uint32_t pixfmt)
{
    v4l2_format fmt{}; 
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pixfmt;
    fmt.fmt.pix.field = V4L2_FIELD_ANY; // ドライバに任せる

    if (xioctl(device_fd_, VIDIOC_TRY_FMT, &fmt) < 0) {
        spdlog::warn("try_format: ioctl failed. errno={}", errno);
        return false;
    }

    bool match_fmt = (fmt.fmt.pix.pixelformat == pixfmt);
    bool match_w   = (fmt.fmt.pix.width == width);
    bool match_h   = (fmt.fmt.pix.height == height);

    if (!match_fmt || !match_w || !match_h) {
        /*
        auto fcc_str = [](uint32_t fcc) {
            std::string s;
            s += static_cast<char>(fcc & 0xFF);
            s += static_cast<char>((fcc >> 8) & 0xFF);
            s += static_cast<char>((fcc >> 16) & 0xFF);
            s += static_cast<char>((fcc >> 24) & 0xFF);
            return s;
        };

        spdlog::warn("try_format mismatch: Req[{} {}x{}] -> Res[{} {}x{}]",
            fcc_str(pixfmt), width, height,
            fcc_str(fmt.fmt.pix.pixelformat), fmt.fmt.pix.width, fmt.fmt.pix.height);
        */
            
        return false;
    }

    return true;
}

void V4L2Capture::drop_frame(int drop_frame_num)
{
    for (int i = 0; i < drop_frame_num; ++i) {
        v4l2_buffer buf{};

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        pollfd poll_fd;

        poll_fd.fd = device_fd_;
        poll_fd.events = POLLIN;

        if (xpoll(&poll_fd, 1, 1000) <= 0) {
            break;
        }

        if (xioctl(device_fd_, VIDIOC_DQBUF, &buf) < 0) {
            break;
        }

        if (xioctl(device_fd_, VIDIOC_QBUF, &buf) < 0) {
            break;
        }
    }
}