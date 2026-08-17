#pragma once

#include <string_view>

#include "command/command.hpp"

/**
 * @brief IM920から抽出済みの16進payloadを意味的なCommandへ復号する。
 *
 * @param payloadHex packet種別から始まる16進文字列。
 * @param command 復号成功時に格納するCommand。
 * @return packet長・種別・Command ID・parameterが正しい場合true。
 */
bool decodeCommand(std::string_view payloadHex, Command& command);
