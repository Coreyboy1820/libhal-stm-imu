// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <libhal-stm-imu/lis3dhtr_i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/i2c.hpp>
// #include <libhal-soft/i2c_bit_bang.hpp>

#include "../hardware_map.hpp"

void application(hardware_map_t& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock;
  auto& console = *p_map.console;
  auto& output_pin0 = *p_map.output_pin0;
  // auto& output_pin1 = *p_map.output_pin1;
  auto& i2c = *p_map.i2c;

  hal::print(console, "Starting lis3dhtr_i2c Application...\n");
  // hal::delay(clock, 50ms);
  auto duration = 0ns;

  // calculating tick_count_for_uptime_calibration
  using period = decltype(duration)::period;

  const auto callibration_start_tick = clock.uptime();
  const auto callibration_end_tick = clock.uptime();
  const auto tick_count_for_uptime = callibration_end_tick - callibration_start_tick;

  // calculating output_pin going to true delay time
  const auto before_output_high = clock.uptime();
  output_pin0.level(true);
  const auto after_output_high = clock.uptime();
  const auto output_true_delay = after_output_high - before_output_high;

  // calculating output_pin going to false delay time
  const auto before_output_low = clock.uptime();
  output_pin0.level(false);
  const auto after_output_low = clock.uptime();
  const auto output_false_delay = after_output_low - before_output_low;

  // calculating ticks required, assuming 50% duty cycle
  const auto frequency = clock.frequency();
  const auto tick_period = hal::wavelength<period>(frequency);
  auto ticks_required = duration / tick_period;
  using unsigned_ticks = std::make_unsigned_t<decltype(ticks_required)>;

  const auto ticks = static_cast<unsigned_ticks>(ticks_required);

  // this would be in the delay

  output_pin0.level(true);
  const auto start_time_high = clock.uptime() - output_true_delay;
  uint64_t uptime = 0;

  const auto ticks_until_timeout_high = ticks + start_time_high;

  while (uptime < ticks_until_timeout_high) {
    uptime = clock.uptime() + tick_count_for_uptime;
    continue;
  }
  // end of delay
  output_pin0.level(false);
  const auto start_time_low = clock.uptime() - output_false_delay;
  uptime = 0;
  
  const auto ticks_until_timeout_low = ticks + start_time_low;

  while (uptime < ticks_until_timeout_low) {
    uptime = clock.uptime() + tick_count_for_uptime;
    continue;
  }
  output_pin0.level(true);
  hal::stm_imu::lis3dhtr_i2c lis(i2c);

  while (true) {
    hal::delay(clock, 500ms);
    auto acceleration = lis.read();
    hal::print<128>(console,
                    "Scale: 2g \t x = %fg, y = %fg, z = %fg \n",
                    acceleration.x,
                    acceleration.y,
                    acceleration.z);
  }
}
