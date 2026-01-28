/**
 * @file resolve_hostname.hpp
 * @brief TCP/UDPクラスの共通処理
 * @author sawada
 * @date 2026-01-28
 * @details mDNSで登録した名前の解決
 */

#ifndef RESOLVE_HOSTNAME_HPP_
#define RESOLVE_HOSTNAME_HPP_

#include <string>
#include <cstring>

std::string resolve_mdns_ipv4(const std::string& hostname);

#endif
