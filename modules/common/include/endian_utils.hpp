#pragma once

#include <array>
#include <cstdint>

int16_t pack_int16_be(uint8_t const *buffer);
uint32_t pack_uint32_be(uint8_t const *buffer);

std::array<uint8_t, 2> unpack_int16_be(int16_t value);
std::array<uint8_t, 4> unpack_uint32_be(uint32_t value);