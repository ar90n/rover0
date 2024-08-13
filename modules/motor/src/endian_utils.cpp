#include <array>

#include "endian_utils.hpp"

int16_t pack_int16_be(uint8_t const* buffer)
{
  return (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
}

uint32_t pack_uint32_be(uint8_t const* buffer)
{
  return (static_cast<uint32_t>(buffer[0]) << 24) | (static_cast<uint32_t>(buffer[1]) << 16) |
         (static_cast<uint32_t>(buffer[2]) << 8) | buffer[3];
}

std::array<uint8_t, 2> unpack_int16_be(int16_t value)
{
  return { static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value) };
}

std::array<uint8_t, 4> unpack_uint32_be(uint32_t value)
{
  return { static_cast<uint8_t>(value >> 24),
           static_cast<uint8_t>(value >> 16),
           static_cast<uint8_t>(value >> 8),
           static_cast<uint8_t>(value) };
}