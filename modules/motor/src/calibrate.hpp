#pragma once

#include "imu.hpp"

namespace calibrate {
imu::Offsets load_offsets();
void calibrate();
bool should_calibrate();
}