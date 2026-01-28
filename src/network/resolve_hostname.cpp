/**
 * @file resolve_hostname.cpp
 * @brief TCP/UDPクラス共通処理
 * @author sawada
 * @date 2026-01-28
 * @details mDNSで登録した名前を解決
 */

#include <stdexcept>
#include <cstring>

#include <arpa/inet.h>
#include <netdb.h>

#include "network/resolve_hostname.hpp"

std::string resolve_mdns_ipv4(const std::string& hostname)
{
    struct addrinfo hints{};
    struct addrinfo* res = nullptr;

    hints.ai_family = AF_INET;
    hints.ai_socktype = 0; 
    hints.ai_flags = AI_ADDRCONFIG;

    int ret = ::getaddrinfo(hostname.c_str(), nullptr, &hints, &res);
    if (ret != 0 || res == nullptr) {
        throw std::runtime_error("name resolution failed: " + std::string(gai_strerror(ret)));
    }

    char ip_str[INET_ADDRSTRLEN];
    auto* addr = reinterpret_cast<sockaddr_in*>(res->ai_addr);

    const char* p = ::inet_ntop(AF_INET, &addr->sin_addr, ip_str, sizeof(ip_str));

    ::freeaddrinfo(res);

    if (!p) {
        throw std::runtime_error("inet_ntop failed");
    }

    return std::string(ip_str);
}
