/**
 * @file json_codec.cpp
 * @brief json parse serialize (implementation)
 * @author sawada
 * @date 2026-01-30
 */

#include "json/json_codec.hpp"
#include <stdexcept>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

void to_json(json& j, const Change_format_args& p)
{
    j = json{
        { "format", p.format }
    };
}

void from_json(const json& j, Change_format_args& p)
{
    j.at("format").get_to(p.format);
}

void to_json(json& j, const shutdown_args& /*p*/)
{
    j = json::object();
}

void from_json(const json& /*j*/, shutdown_args& /*p*/)
{
    ;
}

void to_json(json& j, const type_cmd_json& p)
{
    j["type"] = "cmd";

    if (std::holds_alternative<Change_format_args>(p.args)) {
        j["command"] = "change_format";
        j["args"] = std::get<Change_format_args>(p.args);
    }
    else if (std::holds_alternative<shutdown_args>(p.args)) {
        j["command"] = "shutdown";
        j["args"] = std::get<shutdown_args>(p.args);
    }
}

void from_json(const json& j, type_cmd_json& p)
{
    j.at("type").get_to(p.type);
    j.at("command").get_to(p.command);

    if (p.command == "change_format") {
        p.args = j.at("args").get<Change_format_args>();
    }
    else if (p.command == "shutdown") {
        p.args = j.at("args").get<shutdown_args>();
    }
    else {
        throw std::runtime_error("Unknown command: " + p.command);
    }
}

void to_json(json& j, const yolo_obb& p)
{
    j = json{
        { "center_x", p.center_x },
        { "center_y", p.center_y },
        { "width", p.width },
        { "height", p.height },
        { "rotation_rad", p.rotation_rad }
    };
}

void from_json(const json& j, yolo_obb& p)
{
    j.at("center_x").get_to(p.center_x);
    j.at("center_y").get_to(p.center_y);
    j.at("width").get_to(p.width);
    j.at("height").get_to(p.height);

    if (j.contains("rotation_rad")) {
        j.at("rotation_rad").get_to(p.rotation_rad);
    }
    else if (j.contains("n_rad")) {
        j.at("n_rad").get_to(p.rotation_rad);
    }
    else {
        p.rotation_rad = 0.0f;
    }
}

void to_json(json& j, const detection_data& p)
{
    j = json{
        { "detection_id", p.detection_id },
        { "yolo_obb", p.inference_result }
    };

    if (p.value.has_value()) {
        j["value"] = p.value.value();
    }
    else {
        j["value"] = nullptr;
    }
}

void from_json(const json& j, detection_data& p)
{
    j.at("detection_id").get_to(p.detection_id);

    j.at("yolo_obb").get_to(p.inference_result);

    if (j.contains("value") && !j.at("value").is_null()) {
        p.value = j.at("value").get<std::string>();
    } else {
        p.value = std::nullopt;
    }
}

void to_json(json& j, const type_data_json& p)
{
    j = json{
        { "type", "data" },
        { "frame_id", p.frame_id },
        { "detections", p.detections }
    };
}

void from_json(const json& j, type_data_json& p)
{
    j.at("type").get_to(p.type);
    j.at("frame_id").get_to(p.frame_id);
    j.at("detections").get_to(p.detections);
}

std::string JsonCodec::serialize_cmd(const type_cmd_json& cmd)
{
    json j = cmd;

    return j.dump();
}

std::string JsonCodec::serialize_data(const type_data_json& data)
{
    json j = data;

    return j.dump();
}

JsonCodec::PacketVariant JsonCodec::parse(const std::string& jsonString)
{
    try {
        auto j = json::parse(jsonString);
        std::string typeStr;
        j.at("type").get_to(typeStr);

        if (typeStr == "cmd") {
            return j.get<type_cmd_json>();
        }
        else if (typeStr == "data") {
            return j.get<type_data_json>();
        }
        else {
            throw std::runtime_error("Unknown packet type: " + typeStr);
        }
    }
    catch (const json::parse_error& e) {
        throw std::runtime_error(std::string("JSON parse error: ") + e.what());
    }
    catch (const json::type_error& e) {
        throw std::runtime_error(std::string("JSON type error: ") + e.what());
    }
}
