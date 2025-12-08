/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
  Simulate SensAItion serial IMU/AHRS device
  Generates high-rate sensor data for ArduPilot testing

*/

#pragma once

#include "SIM_SerialDevice.h"
#include <AP_Common/AP_Common.h> 

namespace SITL {

class SensAItion : public SerialDevice {
public:
    SensAItion();
    void update(void);

private:
    void send_packet_0_imu(const struct sitl_fdm &fdm);
    void send_packet_1_orientation(const struct sitl_fdm &fdm);
    void send_packet_2_ins(const struct sitl_fdm &fdm);
    uint32_t calculate_itow(uint64_t now_us, uint32_t start_time_utc);


    void write_packet(uint8_t msg_id, const uint8_t* payload, uint16_t length);
    void write_legacy_packet(const uint8_t* payload, uint16_t length);
    uint16_t calculate_crc(uint8_t msg_id, const uint8_t* payload, uint16_t length, bool use_id);

    uint32_t last_update_us = 0;
    uint32_t tick_count = 0; 
    bool _interleaved_mode = true; // Now mutable, Shall be driven by EAHRS_OPTIONS. TODO
};

} // namespace SITL