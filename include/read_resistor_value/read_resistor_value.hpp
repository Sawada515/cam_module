/**
 * @file read_resistor_value.hpp
 * @brief 抵抗器のROIから抵抗値を推測
 * @author sawada
 * @date 2026-01-30 
 */

#ifndef READ_RESISTOR_VALUE_HPP_
#define READ_RESISTOR_VALUE_HPP_

#include <opencv2/opencv.hpp>

#include <string>
#include <optional>
#include <memory>

class ReadResistorValue {
    public:
        ReadResistorValue(const std::string& band_model_path, const std::string& color_model_path);
        ~ReadResistorValue();

        ReadResistorValue& operator=(const ReadResistorValue&);

        std::optional<double> inference(const cv::Mat& resistor_roi);
    
    private:
        struct Impl;

        std::unique_ptr<Impl> pImpl;
};

#endif
