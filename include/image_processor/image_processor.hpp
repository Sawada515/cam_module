/**
 * @file image_processor.hpp
 * @brief 画像処理クラス
 * @author sawada
 * @date 2026-02-01
 */

#ifndef IMAGE_PROCESSOR_HPP_
#define IMAGE_PROCESSOR_HPP_

#include <string>
#include <cstdint>
#include <vector>
#include <span>

#include "json/json_codec.hpp"
#include "camera/v4l2_capture.hpp"

#include <opencv2/core/mat.hpp>

/**
 * @class ImageProcessor
 * @brief 画像処理クラス
 */
class ImageProcessor {
    public:
        enum class pixel_format : std::uint32_t {
            MJPEG,
            YUV422
        };

        struct raw_image_t {
            uint16_t width;
            uint16_t height;

            pixel_format fmt;

            uint32_t bytesperline;

            size_t valid_size;

            std::span<const uint8_t> data;
        };

        ImageProcessor(const std::string& onnx_model_path, const std::string& band_model_path, const std::string& color_model_path);
        ~ImageProcessor();

        //copyの禁止
        ImageProcessor(const ImageProcessor&) = delete;
        ImageProcessor& operator=(const ImageProcessor&) = delete;

        //moveのみ許可
        ImageProcessor(ImageProcessor&&) noexcept;
        ImageProcessor& operator=(ImageProcessor&&) noexcept;

        void convert_raw_to_bgr(const raw_image_t& frame, cv::Mat& bgr_data);

        bool detect_resistor_coordinate(const cv::Mat& bgr_data, std::vector<detection_data>& result_detect_data);

        void draw_surround_box(cv::Mat& bgr_data, const std::vector<detection_data>& detected_data, const cv::Scalar& color);

        /**
         * @details 精度の関係でV4L2で取得した画像がYUV422のときに呼び出すことを推奨
         */
        void inference_resistor_value(const cv::Mat& bgr_data, const pixel_format fmt, std::vector<detection_data>& result_detect_data);

        /**
         * UDP送信のためにJPEG圧縮する
         */
        bool jpeg_compression_bgr_data(const cv::Mat& bgr_data, std::vector<std::uint8_t>& jpeg_data, std::uint8_t jepg_quality);

    private:
        struct Impl;

        std::unique_ptr<Impl> pImpl;

        void correction_roi_angle(const cv::Mat& bgr_data, cv::Mat& roi, const detection_data& detect_data);
};

#endif
