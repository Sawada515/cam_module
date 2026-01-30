#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <cstdint>
#include <variant>

#include "json/json_client.hpp"

// 受信データの型を判別して表示するためのヘルパー (std::visit用)
struct PacketVisitor
{
    void operator()(const type_cmd_json& cmd)
    {
        std::cout << "[RECV] Command Packet" << std::endl;
        std::cout << "  Type: " << cmd.command << std::endl;

        if (cmd.command == "change_format") {
            if (std::holds_alternative<Change_format_args>(cmd.args)) {
                auto args = std::get<Change_format_args>(cmd.args);
                std::cout << "  Args: Format=" << args.format << std::endl;
            }
        }
    }

    void operator()(const type_data_json& data)
    {
        std::cout << "[RECV] Data Packet" << std::endl;
        std::cout << "  Frame ID: " << data.frame_id << std::endl;
        std::cout << "  Count   : " << data.detections.size() << std::endl;

        for (const auto& item : data.detections) {
            std::string val = item.value.has_value() ? item.value.value() : "null";
            std::cout << "    - ID:" << item.detection_id 
                      << " Val:" << val 
                      << " X:" << item.inference_result.center_x << std::endl;
        }
    }
};

int main()
{
    // TcpThreadではなく、ラッパークラスであるJsonClientを使用
    JsonClient client("gui-module-rpi5.local", 55555);
    //JsonClient client("localhost", 55555);
    client.start();

    std::uint32_t counter = 0;

    while (true) {
        /* ---------- 送信テスト (Data) ---------- */
        {
            // 1. データ構造体を作成
            type_data_json data_packet;
            data_packet.frame_id = counter++;

            // 2. 検出データのダミーを作成 (2個検出したと仮定)
            detection_data det1;
            det1.detection_id = 1;
            det1.value = "1k";
            det1.inference_result = { 100.0f, 200.0f, 50.0f, 20.0f, 0.1f }; // x, y, w, h, rad

            detection_data det2;
            det2.detection_id = 2;
            det2.value = std::nullopt; // 読み取れなかった場合
            det2.inference_result = { 300.0f, 400.0f, 60.0f, 30.0f, -0.2f };

            detection_data det3;
            det2.detection_id = 2;
            det2.value = std::nullopt; // 読み取れなかった場合
            det2.inference_result = { 1300.0f, 4200.0f, 160.0f, 90.0f, -0.1115f };

            detection_data det4;
            det2.detection_id = 2;
            det2.value = std::nullopt; // 読み取れなかった場合
            det2.inference_result = { 30000.0f, 1400.0f, 610.0f, 40.0f, -0.55551f };

            data_packet.detections.push_back(det1);
            data_packet.detections.push_back(det2);
            data_packet.detections.push_back(det3);
            data_packet.detections.push_back(det4);

            client.send_data(data_packet);

            std::cout << "[SEND] Data Frame: " << data_packet.frame_id << std::endl;
        }

        while (true) {
            std::optional<JsonClient::recv_cmd_data> recv_json_opt = client.try_receive();
            
            if (!recv_json_opt.has_value()) {
                break;
            }

            JsonClient::recv_cmd_data recv_data = recv_json_opt.value();

            if (recv_data.cmd == JsonClient::cmd_kinds::CHANGE_FORMAT) {
                if (recv_data.args == JsonClient::args_kinds::MJPEG) {
                    std::cout << "recv cmd change_format args MJPG" << std::endl;
                }
                else if (recv_data.args == JsonClient::args_kinds::YUV422) {
                    std::cout << "recv cmd change_format args YUV422" << std::endl;
                }
                else {
                    std::cout << "recv cmd change_format args None" << std::endl;
                }
            }
            else if (recv_data.cmd == JsonClient::cmd_kinds::SHUTDOWN) {
                std::cout << "recv cmd shutdown" << std::endl;
            }
            else {
                std::cout << "Unknown command" << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    client.stop();
    return 0;
}
