#pragma once

#include <cstdint>
#include <functional>
#include <optional>

namespace transport {
using Writer = std::function<void(const uint8_t *, size_t)>;
void init(Writer &&writer);
void send(uint32_t data);
void reset();
std::optional<uint32_t> consume(uint8_t byte);
} // namespace transport
