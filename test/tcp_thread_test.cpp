#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <cstdint>

#include "network/tcp_thread.hpp"

int main()
{
    TcpThread tcp("gui-module-rpi5.local", 55555);
    tcp.start();

    std::uint32_t counter = 0;

    while (true) {
        /* ---------- 送信テスト ---------- */
        {
            std::string json =
                "{"
                "\"type\":\"test\","
                "\"counter\":" + std::to_string(counter++) +
                "}";

            std::vector<std::uint8_t> data(json.begin(), json.end());
            tcp.send(std::move(data));
        }

        /* ---------- 受信テスト ---------- */
        {
            std::vector<std::uint8_t> recv_data;
            while (tcp.recv(recv_data)) {
                std::string json(recv_data.begin(), recv_data.end());
                std::cout << "[RECV] " << json << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    tcp.stop();
    return 0;
}