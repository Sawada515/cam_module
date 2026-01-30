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

            data_packet.detections.push_back(det1);
            data_packet.detections.push_back(det2);

            // 3. 送信 (シリアライズとヘッダ付与は内部で行われる)
            client.send_data(data_packet);

            std::cout << "[SEND] Data Frame: " << data_packet.frame_id << std::endl;
        }

        /* ---------- 送信テスト (Command - 5回に1回送る例) ---------- */
        if (counter % 5 == 0) {
            type_cmd_json cmd_packet;
            cmd_packet.command = "change_format";
            cmd_packet.args = Change_format_args{ "MJPEG" };

            client.send_command(cmd_packet);
            std::cout << "[SEND] Command: change_format" << std::endl;
        }

        /* ---------- 受信テスト ---------- */
        // ノンブロッキングで受信キューを確認
        while (true) {
            auto packet_opt = client.try_receive();
            
            if (!packet_opt.has_value()) {
                break;
            }

            // 受信データがあれば std::visit で型ごとに処理
            std::visit(PacketVisitor{}, packet_opt.value());
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    client.stop();
    return 0;
}
