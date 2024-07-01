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

#ifndef XV11LIDAR_H
#define XV11LIDAR_H

#include "mamePID.hpp"

#include <array>
#include <functional>
#include <stdint.h>

#define XV11LIDAR_PACKET_SIZE 22
#define LIDAR_SAMPLE_TIME_MS 100

template <typename T, typename U>
concept Component = requires(T t) {
  { t.calculate(U{}, U{}) } -> std::same_as<U>;
};

namespace xv11 {
struct DataPacket {
  uint32_t timestamp_us; // timestamp in microseconds
  uint8_t angle_quad;    // 0-89 for readings 0-4 356-359
  uint16_t motor_rpm;    // speed in rpm
  uint16_t distances[4]; // flags and distance or error code
  uint16_t signals[4];   // signal strengths
};

using ReturnType = std::tuple<bool, uint8_t>;
using DataReader = std::function<ReturnType()>;
using PWMWriter = std::function<void(float)>;
using TimestampGetter = std::function<uint32_t()>;
using PacketBuffer = std::array<uint8_t, XV11LIDAR_PACKET_SIZE>;

class Lidar {
public:
  /* Setup */
  Lidar(DataReader read_byte, PWMWriter write_pwm, TimestampGetter micros,
        float motor_rpm);

  /* Cyclic */
  bool process(DataPacket *packet);
  void apply_motor_pid(float motor_rpm);

private:
  /* IO */
  DataReader m_read_byte;
  PWMWriter m_write_pwm;
  TimestampGetter m_micros;

  /* Packet & Decoding */
  PacketBuffer m_packet;
  uint32_t m_packet_bytes;
  uint32_t m_packet_timestamp_us;

  /* Motor control */
  const float motor_rpm;
  uint32_t last_process_time = 0;
  float cv = 0.0f;
  using PIDType = decltype(mamePID::pi<float>(0.0, 0.0, LIDAR_SAMPLE_TIME_MS));
  PIDType m_motor_pid =
      mamePID::pi<float>(1.8, 0.00025, LIDAR_SAMPLE_TIME_MS, -255.0, 255.0);
};
} // namespace xv11

#endif
