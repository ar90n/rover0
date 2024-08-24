#pragma once

#include "hardware/i2c.h"

#include "endian_utils.hpp"
#include "gpio.hpp"

namespace imu {
struct Offsets
{
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
};

template<unsigned M, unsigned SDA_PIN, unsigned SCL_PIN>
class IMU_6050
{
public:
  using GpioSda = Gpio<SDA_PIN, GPIO_FUNC_I2C>;
  using GpioScl = Gpio<SCL_PIN, GPIO_FUNC_I2C>;

  constexpr static int I2C_ADDR = 0x68;

  struct Accel
  {
    int16_t x;
    int16_t y;
    int16_t z;
  };

  struct Gyro
  {
    int16_t x;
    int16_t y;
    int16_t z;
  };

  struct Result
  {
    Accel   accel;
    Gyro    gyro;
    int16_t temp;
  };

  inline static IMU_6050& instance()
  {
    static IMU_6050 instance;
    return instance;
  }

  void init(uint baudrate, Offsets const& offsets)
  {
    i2c_init(inst(), baudrate);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    uint8_t const buf[]{
      0x6B, 0x00,
      //0x06, static_cast<uint8_t>(offsets.acc_x >> 8),
      //0x07, static_cast<uint8_t>(offsets.acc_x & 0xff),
      //0x08, 0x00,//static_cast<uint8_t>(offsets.acc_y >> 8),
      //0x09, 0x00,//static_cast<uint8_t>(offsets.acc_y & 0xff),
      //0x0a, 0x00,//static_cast<uint8_t>(offsets.acc_z >> 8),
      //0x0b, 0x00,//static_cast<uint8_t>(offsets.acc_z & 0xff),
      //0x13, static_cast<uint8_t>(offsets.gyro_x >> 8),
      //0x14, static_cast<uint8_t>(offsets.gyro_x & 0xff),
      //0x15, static_cast<uint8_t>(offsets.gyro_y >> 8),
      //0x16, static_cast<uint8_t>(offsets.gyro_y & 0xff),
      //0x17, static_cast<uint8_t>(offsets.gyro_z >> 8),
      //0x18, static_cast<uint8_t>(offsets.gyro_z & 0xff),
    };
    i2c_write_blocking(inst(), I2C_ADDR, buf, sizeof buf, false);
  }

  void write(uint8_t address, uint8_t value) { i2c_write_blocking(inst(), address, &value, 1, false); }

  Result read()
  {
    return Result{
      .accel = read_accel(),
      .gyro  = read_gyro(),
      .temp  = read_temp(),
    };
  }

private:
  static i2c_inst_t* inst()
  {
    static_assert(M == 0 || M == 1, "Invalid I2C instance");
    return i2c_get_instance(M);
  }

  Accel read_accel()
  {
    i2c_write_blocking(inst(), I2C_ADDR, reinterpret_cast<uint8_t const*>("\x3B"), 1, true);

    uint8_t buffer[6];
    i2c_read_blocking(inst(), I2C_ADDR, buffer, 6, false);

    return Accel{
      .x = pack_int16_be(buffer + 0),
      .y = pack_int16_be(buffer + 2),
      .z = pack_int16_be(buffer + 4),
    };
  }

  Gyro read_gyro()
  {
    i2c_write_blocking(inst(), I2C_ADDR, reinterpret_cast<uint8_t const*>("\x43"), 1, true);

    uint8_t buffer[6];
    i2c_read_blocking(inst(), I2C_ADDR, buffer, 6, false);

    return Gyro{
      .x = pack_int16_be(buffer + 0),
      .y = pack_int16_be(buffer + 2),
      .z = pack_int16_be(buffer + 4),
    };
  }

  int16_t read_temp()
  {
    i2c_write_blocking(inst(), I2C_ADDR, reinterpret_cast<uint8_t const*>("\x41"), 1, true);

    uint8_t buffer[2];
    i2c_read_blocking(inst(), I2C_ADDR, buffer, 2, false);

    return pack_int16_be(buffer);
  }

  IMU_6050() {}
};

} // namespace imu
