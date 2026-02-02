/**
 * @file image_processor.cpp
 * @brief 画像処理クラス
 * @author sawada
 * @date 2026-02-01
 */

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <turbojpeg.h>

#include <optional>

#include "logger/logger.hpp"
#include "json/json_codec.hpp"
#include "image_processor/image_processor.hpp"
#include "read_resistor_value/read_resistor_value.hpp"

namespace {
    namespace YoloIndex {
        constexpr int CX = 0;
        constexpr int CY = 1;
        constexpr int W = 2;
        constexpr int H = 3;
        //constexpr int ANGLE = 4;
        //constexpr int SCORE = 5; 

        constexpr int SCORE = 4; 
        constexpr int ANGLE = 5;
        
        constexpr int MIN_FEATURES = 6;
        constexpr int MAX_FEATURES = 7; 
    }

    constexpr float CONFIDENCE_THRESHOLD = 0.5f;
    constexpr float NMS_THRESHOLD = 0.4f;

    constexpr int MODEL_INPUT_WIDTH = 960;
}

struct ImageProcessor::Impl {
    cv::dnn::Net net;

    std::unique_ptr<ReadResistorValue> resistor_reader;
    
    tjhandle jpeg_compressor = nullptr;

    Impl(const std::string& onnx_path, const std::string& band_path, const std::string& color_path) {
        try {
            net = cv::dnn::readNetFromONNX(onnx_path);

            if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
                net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            }
            else {
                net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            }
        }
        catch (const cv::Exception& e) {
            spdlog::error("Failed to model load error {}", e.what());
        }

        try {
            resistor_reader = std::make_unique<ReadResistorValue>(band_path, color_path);
        }
        catch (const std::exception& e) {
            spdlog::error("Failed to initialize ReadResistorValue {}", e.what());
        }

        jpeg_compressor = tjInitCompress();
        if (!jpeg_compressor) {
            spdlog::error("Failed to Turbojpeg init");
        }
    }

    ~Impl() {
        if (jpeg_compressor) {
            tjDestroy(jpeg_compressor);
        }
    }
};

ImageProcessor::ImageProcessor(const std::string& onnx_model_path, const std::string& band_model_path, const std::string& color_model_path)
    : pImpl(std::make_unique<Impl>(onnx_model_path, band_model_path, color_model_path)) 
{
    ;
}

ImageProcessor::~ImageProcessor() = default;

ImageProcessor::ImageProcessor(ImageProcessor&&) noexcept = default;
ImageProcessor& ImageProcessor::operator=(ImageProcessor&&) noexcept = default;

void ImageProcessor::convert_raw_to_bgr(const raw_image_t& frame, cv::Mat& bgr_data)
{
    if (frame.data.empty()) {
        return;
    }

    cv::Mat raw_data(1, frame.data.size(), CV_8UC1, const_cast<unsigned char*>(frame.data.data()));

    if (frame.fmt == pixel_format::MJPEG) {
        cv::imdecode(raw_data, cv::IMREAD_COLOR, &bgr_data);
    } 
    else if (frame.fmt == pixel_format::YUV422) {
        cv::Mat raw_wrapper(frame.height, frame.width, CV_8UC2, raw_data.data);
        cv::cvtColor(raw_wrapper, bgr_data, cv::COLOR_YUV2BGR_YUYV);
    }
}

bool ImageProcessor::detect_resistor_coordinate(const cv::Mat& bgr_data, std::vector<detection_data>& result_detect_data)
{
    if (bgr_data.empty() || pImpl->net.empty()) {
        return false;
    }

    cv::Mat blob;
    cv::dnn::blobFromImage(bgr_data, blob, 1.0 / 255.0, cv::Size(MODEL_INPUT_WIDTH, MODEL_INPUT_WIDTH), cv::Scalar(), true, false);
    pImpl->net.setInput(blob);

    std::vector<cv::Mat> outputs;
    pImpl->net.forward(outputs, pImpl->net.getUnconnectedOutLayersNames());
    
    if (outputs.empty()) {
        return false;
    }

    const cv::Mat& output = outputs[0];
    CV_Assert(output.dims == 3); 

    const int dim1 = output.size[1];
    const int dim2 = output.size[2];
    
    std::vector<cv::RotatedRect> boxes_rotated;
    std::vector<cv::Rect> boxes_rect;
    std::vector<float> confidences;

    float scale_x = (float)bgr_data.cols / static_cast<float>(MODEL_INPUT_WIDTH);
    float scale_y = (float)bgr_data.rows / static_cast<float>(MODEL_INPUT_WIDTH);

    auto add_detection = [&](float cx, float cy, float w, float h, float angle_rad, float score) {
        float angle_deg = angle_rad * (180.0f / CV_PI);
        cv::RotatedRect rrect(cv::Point2f(cx, cy), cv::Size2f(w, h), angle_deg);
        
        boxes_rotated.push_back(rrect);
        boxes_rect.push_back(rrect.boundingRect()); // NMS用
        confidences.push_back(score);
    };

    if (dim2 >= YoloIndex::MIN_FEATURES && dim2 <= YoloIndex::MAX_FEATURES) {
        const int num_anchors = dim1;
        for (int i = 0; i < num_anchors; ++i) {
            const float* row = output.ptr<float>(0, i);
            float score = row[YoloIndex::SCORE];
            if (score >= CONFIDENCE_THRESHOLD) {
                add_detection(
                    row[YoloIndex::CX] * scale_x,
                    row[YoloIndex::CY] * scale_y,
                    row[YoloIndex::W]  * scale_x,
                    row[YoloIndex::H]  * scale_y,
                    row[YoloIndex::ANGLE],
                    score
                );
            }
        }
    }
    else if (dim1 >= YoloIndex::MIN_FEATURES && dim1 <= YoloIndex::MAX_FEATURES) {
        const int num_anchors = dim2;
        const float* p_cx    = output.ptr<float>(0, YoloIndex::CX);
        const float* p_cy    = output.ptr<float>(0, YoloIndex::CY);
        const float* p_w     = output.ptr<float>(0, YoloIndex::W);
        const float* p_h     = output.ptr<float>(0, YoloIndex::H);
        const float* p_angle = output.ptr<float>(0, YoloIndex::ANGLE);
        const float* p_score = output.ptr<float>(0, YoloIndex::SCORE);

        for (int i = 0; i < num_anchors; ++i) {
            float score = p_score[i];
            if (score >= CONFIDENCE_THRESHOLD) {
                add_detection(
                    p_cx[i] * scale_x,
                    p_cy[i] * scale_y,
                    p_w[i]  * scale_x,
                    p_h[i]  * scale_y,
                    p_angle[i],
                    score
                );
            }
        }
    }
    else {
        spdlog::error("Unexpected output shape {}, {}", dim1, dim2);
        return false;
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes_rect, confidences, CONFIDENCE_THRESHOLD, NMS_THRESHOLD, indices);

    result_detect_data.clear();
    int id_counter = 0;

    for (int idx : indices) {
        const auto& box = boxes_rotated[idx];
        
        detection_data det;
        det.detection_id = id_counter++;
        
        det.inference_result.center_x = box.center.x;
        det.inference_result.center_y = box.center.y;
        det.inference_result.width = box.size.width;
        det.inference_result.height = box.size.height;
        det.inference_result.rotation_rad = box.angle * (CV_PI / 180.0f); 

        det.value = std::nullopt;
        result_detect_data.push_back(det);
    }

    return !result_detect_data.empty();
}

void ImageProcessor::draw_surround_box(cv::Mat& bgr_data, const std::vector<detection_data>& detected_data, const cv::Scalar& color)
{
    for (const auto& det : detected_data) {
        float angle_deg = det.inference_result.rotation_rad * (180.0f / CV_PI);
        cv::RotatedRect rect(
            cv::Point2f(det.inference_result.center_x, det.inference_result.center_y),
            cv::Size2f(det.inference_result.width, det.inference_result.height),
            angle_deg
        );

        cv::Point2f vertices[4];
        rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(bgr_data, vertices[i], vertices[(i + 1) % 4], color, 2);
        }
    }
}

void ImageProcessor::inference_resistor_value(const cv::Mat& bgr_data, const pixel_format fmt, std::vector<detection_data>& result_detect_data)
{
    if (fmt == ImageProcessor::pixel_format::MJPEG) {
        spdlog::warn("YUV422 is recommended for precision.");
    }

    cv::Mat roi;
    for (auto& det : result_detect_data) {
        correction_roi_angle(bgr_data, roi, det);

        if (roi.empty()) {
            continue;
        }

        auto result = pImpl->resistor_reader->inference(roi);

        if (result.has_value()) {
            det.value = std::to_string(result.value());
        }
        else {
            det.value = std::nullopt;
        }
    }
}

bool ImageProcessor::jpeg_compression_bgr_data(const cv::Mat& bgr_data, std::vector<std::uint8_t>& jpeg_data, std::uint8_t jpeg_quality)
{
    if (!pImpl->jpeg_compressor) {
        return false;
    }

    unsigned long jpeg_size = 0;
    unsigned char* jpeg_buffer = nullptr;

    int pitch = static_cast<int>(bgr_data.step);

    int res = tjCompress2(
        pImpl->jpeg_compressor, 
        bgr_data.data, 
        bgr_data.cols, 
        pitch,
        bgr_data.rows, 
        TJPF_BGR, 
        &jpeg_buffer, 
        &jpeg_size, 
        TJSAMP_444, 
        jpeg_quality, 
        TJFLAG_FASTDCT
    );

    if (res == 0) {
        try {
            jpeg_data.assign(jpeg_buffer, jpeg_buffer + jpeg_size);
        }
        catch (...) {
            tjFree(jpeg_buffer);

            return false;
        }
        tjFree(jpeg_buffer);

        return true;
    }
    else {
        spdlog::error("Filaed to Turbojepg compress {}", std::string(tjGetErrorStr2(pImpl->jpeg_compressor)));

        return false;
    }
}

void ImageProcessor::correction_roi_angle(const cv::Mat& bgr_data, cv::Mat& roi, const detection_data& detect_data)
{
    const auto& obb = detect_data.inference_result;

    float angle_deg = obb.rotation_rad * (180.0f / CV_PI);
    cv::RotatedRect rect(
        cv::Point2f(obb.center_x, obb.center_y),
        cv::Size2f(obb.width, obb.height),
        angle_deg
    );

    cv::Point2f pts[4];
    rect.points(pts);

    cv::Point2f tl, tr, br, bl;

    float min_s = std::numeric_limits<float>::max();
    float max_s = std::numeric_limits<float>::lowest();
    float min_diff = std::numeric_limits<float>::max();
    float max_diff = std::numeric_limits<float>::lowest();

    for (int i = 0; i < 4; i++) {
        float s = pts[i].x + pts[i].y;
        float diff = pts[i].y - pts[i].x;

        // 左上(tl): s が最小
        if (s < min_s) {
            min_s = s;
            tl = pts[i];
        }
        // 右下(br): s が最大
        if (s > max_s) {
            max_s = s;
            br = pts[i];
        }
        // 右上(tr): diff (y-x) が最小
        if (diff < min_diff) {
            min_diff = diff;
            tr = pts[i];
        }
        // 左下(bl): diff (y-x) が最大
        if (diff > max_diff) {
            max_diff = diff;
            bl = pts[i];
        }
    }

    auto euclidean_dist = [](cv::Point2f p1, cv::Point2f p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    };

    float widthA = euclidean_dist(br, bl);
    float widthB = euclidean_dist(tr, tl);
    int maxWidth = std::max(static_cast<int>(widthA), static_cast<int>(widthB));

    float heightA = euclidean_dist(tr, br);
    float heightB = euclidean_dist(tl, bl);
    int maxHeight = std::max(static_cast<int>(heightA), static_cast<int>(heightB));

    std::vector<cv::Point2f> src_pts = {tl, tr, br, bl};
    std::vector<cv::Point2f> dst_pts = {
        cv::Point2f(0, 0),
        cv::Point2f(maxWidth - 1, 0),
        cv::Point2f(maxWidth - 1, maxHeight - 1),
        cv::Point2f(0, maxHeight - 1)
    };

    cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
    cv::warpPerspective(bgr_data, roi, M, cv::Size(maxWidth, maxHeight));

    if (!roi.empty() && roi.rows > roi.cols) {
        cv::rotate(roi, roi, cv::ROTATE_90_CLOCKWISE);
    }

    if (!roi.empty()) {
        cv::imwrite("debug.bmp", roi);

        spdlog::info("save debug image");
    }
}
