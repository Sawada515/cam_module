/**
 * @file v4l2_capture.hpp
 * @brief V4L2 キャプチャ実装
 * @author sawada
 * @date 2026-01-24
 */

#ifndef V4L2_CAPTURE_HPP_
#define V4L2_CAPTURE_HPP_

#include <linux/videodev2.h>

#include <cstdint>
#include <atomic>
#include <string>
#include <vector>
#include <stdexcept>

/**
 * @class V4L2Capture
 * @brief V4L2を使ったカメラデバイスの操作
 * @details スレッドから呼び出すこと
 * @note スレッドセーフではないので、呼び出し側が保証するこ
 */
class V4L2Capture {
    public:
        /**
         * @enum frame_format
         * @brief カメラフォーマット
         */
        enum class frame_format : std::uint32_t {
            YUV422 = V4L2_PIX_FMT_YUYV,
            MJPEG = V4L2_PIX_FMT_MJPEG
        };

        /**
         * @struct Frame
         * @brief フレームデータ保持用
         * @details データはYUV422のサイズを基準に作成しておく
         */
        struct Frame {
            std::uint16_t width;
            std::uint16_t height;
            
            size_t size;

            std::uint32_t bytesperline;

            frame_format fmt;

            std::vector<std::uint8_t> data;
        };

        V4L2Capture(std::string device_file_name, std::uint16_t width, std::uint16_t height, frame_format fmt);

        ~V4L2Capture();

        /**
         * @brief フレームの取得
         */
        bool capture_frame(Frame& frame);

        /**
         * @brief ストリームのON
         */
        void stream_on();

        /**
         * @brief ストリームのOFF
         */
        void stream_off();

        /**
         * @brief フォーマットの変更
         * @details スレッド側で必ず例外をキャッチし、例外発生時はオブジェクトを破棄し、再生成すること
         * @note この関数はcapture_frame()が呼ばれていない状態のみで呼び出し可能
         */
        void reconfigure(frame_format fmt);

    private:
        struct mmap_buffer {
            void* start = nullptr;

            size_t length = 0;
        };

        std::string device_file_name_;

        std::uint16_t width_;
        std::uint16_t height_;

        std::uint32_t bytesperline_;

        frame_format fmt_;

        int device_fd_;

        static constexpr std::uint16_t CAPTURE_BUFFER_COUNT = 4;

        std::vector<struct mmap_buffer> buffers_;

        int open_device();
        void close_device();

        void request_capture_buffer();

        void set_frame_format();

        void cleanup_buffers();

        bool try_format(std::uint16_t width, std::uint16_t height, uint32_t pixfmt);

        /**
         * @details 白飛び対策としてフォーマット変更直後数フレーム捨てる
         */
        void drop_frame(int drop_frame_num);
};

#endif
