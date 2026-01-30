/**
 * @file json_codec.hpp
 * @brief json parse serialize
 * @author sawada
 * @date 2026-01-30
 */

#ifndef JSON_CODEC_HPP_
#define JSON_CODEC_HPP_

#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>
#include <variant>
#include <vector>
#include <optional>

using json = nlohmann::json;

enum class type : std::uint8_t {
    CMD,
    DATA
};

struct Change_format_args {
    std::string format;
};

struct shutdown_args {
};

struct type_cmd_json {
    std::string type;
    std::string command;

    std::variant<Change_format_args, shutdown_args> args;
};

struct yolo_obb {
    float center_x;
    float center_y;
    float width;
    float height;

    float rotation_rad;
};

struct detection_data {
    int detection_id;

    std::optional<std::string> value;

    yolo_obb inference_result;
};

struct type_data_json {
    std::string type;

    int frame_id;

    std::vector<detection_data> detections;
};

void to_json(json& j, const Change_format_args& p);
void from_json(const json& j, Change_format_args& p);

void to_json(json& j, const shutdown_args& p);
void from_json(const json& j, shutdown_args& p);

void to_json(json& j, const type_cmd_json& p);
void from_json(const json& j, type_cmd_json& p);

void to_json(json& j, const yolo_obb& p);
void from_json(const json& j, yolo_obb& p);

void to_json(json& j, const detection_data& p);
void from_json(const json& j, detection_data& p);

void to_json(json& j, const type_data_json& p);
void from_json(const json& j, type_data_json& p);

class JsonCodec {
    public:
        using PacketVariant = std::variant<type_cmd_json, type_data_json>;

        static std::string serialize_cmd(const type_cmd_json& cmd);
        static std::string serialize_data(const type_data_json& data);

        static PacketVariant parse(const std::string& jsonString);
};

#endif
