#include <hardware/flash.h>

#include "device.hpp"
#include "gpio.hpp"
#include "imu.hpp"
#include "logger.hpp"

namespace {
auto gpio_led = device::GpioLED::instance();
auto gpio_esw = device::GpioESW::instance();
auto imu_     = device::IMU::instance();

void store_sensor_offsets(imu::Offsets const& offsets)
{
  const uint32_t FLASH_TARGET_OFFSET = 0x1F0000;
  uint8_t        write_data[FLASH_PAGE_SIZE];
  *reinterpret_cast<uint64_t*>(write_data)                 = 0xdeadbeef;
  *reinterpret_cast<imu::Offsets*>(write_data + 8) = offsets;

  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(FLASH_TARGET_OFFSET, write_data, FLASH_PAGE_SIZE);
  restore_interrupts(ints);
}

bool is_timeout(absolute_time_t end_time)
{
  return to_ms_since_boot(get_absolute_time()) >= to_ms_since_boot(end_time);
}

int16_t round_div(int32_t const dividend, uint32_t const divisor)
{
  return static_cast<int16_t>((dividend + divisor / 2) / divisor);
}
}

namespace calibrate {

imu::Offsets load_offsets()
{
  const uint32_t FLASH_TARGET_OFFSET = 0x1F0000;
  auto const     magic{ *(uint64_t*)(XIP_BASE + FLASH_TARGET_OFFSET + 8) };
  if (magic != 0xdeadbeef) {
    return {
      .acc_x  = 0,
      .acc_y  = 0,
      .acc_z  = 0,
      .gyro_x = 0,
      .gyro_y = 0,
      .gyro_z = 0,
    };
  }

  return *(const imu::Offsets*)(XIP_BASE + FLASH_TARGET_OFFSET + 8);
}
void calibrate()
{
  logger::log("** Calibration started **");

  logger::log(" Capturing data for calibration");
  int32_t    agg_acc_x{ 0 };
  int32_t    agg_acc_y{ 0 };
  int32_t    agg_acc_z{ 0 };
  int32_t    agg_gyr_x{ 0 };
  int32_t    agg_gyr_y{ 0 };
  int32_t    agg_gyr_z{ 0 };
  uint32_t   capture_count{ 0 };
  auto const capturing_timeout{ make_timeout_time_ms(3000) };
  while (!is_at_the_end_of_time(capturing_timeout)) {
    auto const result{ imu_.read() };
    agg_acc_x += result.accel.x;
    agg_acc_y += result.accel.y;
    agg_acc_z += result.accel.z;
    agg_gyr_x += result.gyro.x;
    agg_gyr_y += result.gyro.y;
    agg_gyr_z += result.gyro.z;
    capture_count++;
  }

  logger::log(" Store calibration data");

  store_sensor_offsets({
    .acc_x  = round_div(agg_acc_x, capture_count),
    .acc_y  = round_div(agg_acc_y, capture_count),
    .acc_z  = round_div(agg_acc_z, capture_count),
    .gyro_x = round_div(agg_gyr_x, capture_count),
    .gyro_y = round_div(agg_gyr_y, capture_count),
    .gyro_z = round_div(agg_gyr_z, capture_count),
  });

  logger::log(" Calibration completed");
}

bool should_calibrate()
{
  bool     last_esw_state{ gpio_esw.read() };
  uint32_t esw_toggle_count{ 0 };

  auto checking_timeout{ make_timeout_time_ms(1000) };
  while (!is_timeout(checking_timeout)) {
    auto const esw_state{ gpio_esw.read() };
    if (esw_state != last_esw_state) {
      esw_toggle_count++;
    }
    last_esw_state = esw_state;

    if (esw_toggle_count >= 3) {
      return true;
    }
    sleep_ms(10);
  }

  return false;
}
} // namespace calibrate