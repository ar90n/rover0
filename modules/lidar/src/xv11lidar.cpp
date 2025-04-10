/*
 * xv11lidar-arduino library header
 *
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 * Copyright 2024 (C) Masahiro Wada <argon.argon.argon@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */

#include "xv11lidar.h"

namespace
{
using namespace ::xv11;

constexpr uint8_t StartByte       = 0xFA;
constexpr float   SpeedMultiplier = 1.0f / 64.0f;

inline uint16_t decode_u16(uint8_t const* const data)
{
  return static_cast<uint16_t>(data[1]) << 8 | data[0];
}

inline uint16_t checksum(PacketBuffer const& data)
{
  uint32_t       chk32 = 0;
  uint8_t const* ptr   = data.data();
  for (int i = 0; i < 10; ++i, ptr += 2)
  {
    uint16_t const word = decode_u16(ptr);
    chk32               = (chk32 << 1) + word;
  }

  return ((chk32 & 0x7fff) + (chk32 >> 15)) & 0x7FFF;
}

inline void decodePacket(PacketBuffer const& data, uint32_t const timestamp_us, DataPacket* const packet)
{
  uint8_t const* ptr = data.data();

  //*ptr == StartByte;
  ptr++;

  packet->angle_quad = *ptr - 0xA0;
  ptr++;

  packet->motor_rpm  = static_cast<float>(decode_u16(ptr)) * SpeedMultiplier;
  ptr               += 2;

  for (int i = 0; i < 4; ++i)
  {
    packet->distances[i]  = decode_u16(ptr);
    ptr                  += 2;

    packet->signals[i]  = decode_u16(ptr);
    ptr                += 2;
  }
  packet->timestamp_us = timestamp_us;
}

inline bool is_valid_packet(PacketBuffer const& data)
{
  uint16_t const OffsetCRC = 20;
  uint16_t const crc       = decode_u16(data.data() + OffsetCRC);
  return checksum(data) == crc;
}
} // namespace

xv11::Lidar::Lidar(DataReader read_byte, PWMWriter write_pwm, TimestampGetter micros, float target_motor_rpm)
  : m_read_byte(read_byte)
  , m_write_pwm(write_pwm)
  , m_micros(micros)
  , m_packet_bytes(0)
  , target_motor_rpm(target_motor_rpm)
{
}

bool xv11::Lidar::process(DataPacket* packet)
{
  bool has_data = false;

  do
  {
    has_data = false;
    while (m_packet_bytes < std::tuple_size<PacketBuffer>::value)
    {
      const auto ret = m_read_byte();
      if (!std::get<0>(ret))
      {
        break;
      }
      int byte = std::get<1>(ret);

      if (m_packet_bytes == 0)
      {
        if (byte != StartByte)
        {
          continue;
        }

        m_packet_timestamp_us = m_micros();
      }
      m_packet[m_packet_bytes++] = byte;
    }

    if (m_packet_bytes == std::tuple_size<PacketBuffer>::value)
    {
      has_data       = true;
      m_packet_bytes = 0;
      if (is_valid_packet(m_packet))
      {
        decodePacket(m_packet, m_packet_timestamp_us, packet);
        cur_motor_rpm = packet->motor_rpm;
        return true;
      }
    }
  } while (has_data);
  return false;
}

void xv11::Lidar::apply_motor_pid()
{
  const auto cur_time    = m_micros();
  const auto duration_ms = (cur_time - last_process_time) / 1000;
  if (duration_ms < LIDAR_SAMPLE_TIME_MS)
  {
    return;
  }
  last_process_time = cur_time;

  auto const delta = m_motor_pid.calculate(target_motor_rpm, cur_motor_rpm);
  pwm_ratio        = std::clamp(pwm_ratio + delta, 0.8f, 0.95f); // pwm ratio should be 0.8f ~ 0.95f
  m_write_pwm(pwm_ratio);
}