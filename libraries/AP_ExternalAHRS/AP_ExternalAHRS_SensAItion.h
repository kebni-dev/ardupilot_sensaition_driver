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

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SENSAITION_ENABLED

#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_SensAItion : public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_SensAItion(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &_state);

    // get serial port number for the uart, or -1 if not applicable
    int8_t get_port() const override;

    // accessors for AP_AHRS
    bool healthy() const override;
    bool initialised() const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    
    // get model/type name
    const char* get_name() const override;

    void update() override;

    uint8_t num_gps_sensors() const override
    {
        return 0;  // SensAItion supports AHRS and IMU for ardupilot as of now, INS is coming
    }

    // Configuration Mode Selection
    // ============================
    // SensAItion can operate in the following modes:
    // CONFIG_MODE_IMU:  Provides accel, gyro, mag, baro (36 data bytes + header + checksum = 38 total)
    // CONFIG_MODE_AHRS: Provides IMU data + quaternion attitude (52 data bytes + header + checksum = 54 total)
    // CONFIG_MODE_INS:  Not yet supported, is coming later
    //
    // Future: Will become AP_Param configurable via Mission Planner
    enum class ConfigMode {
        CONFIG_MODE_IMU = 0,   // IMU only mode
        CONFIG_MODE_AHRS = 1   // AHRS mode
    };

private:
    // SensAItion protocol constants
    static constexpr uint8_t HEADER_BYTE = 0xFA;
    static constexpr int MAX_PACKET_SIZE = 256;
    
    // Packet parsing state machine.
    // Should only be accessed from inside the thread to avoid data races.
    enum class ParseState { LOOKING_FOR_HEADER, COLLECTING_PACKET, };
    ParseState parse_state_ = LOOKING_FOR_HEADER;
    uint8_t *packet_buffer_;
    uint16_t packet_buffer_len_ = 0;

    // SensAItion parser methods
    uint8_t calculate_xor_checksum(const uint8_t* data, size_t start, size_t length);
    bool extract_sensor_data(const uint8_t* packet);
    
    // Parsing methods
    // REVIEW: Document what the return bool means
    bool parse_packet(const uint8_t* packet, size_t packet_size);
    bool parse_multiple_bytes(const uint8_t* data, size_t data_size);

    // ArduPilot integration
    AP_HAL::UARTDriver *uart;
    uint32_t baudrate;
    int8_t port_num;

    // Static config for now - will become AP_Param later
    static constexpr ConfigMode DEFAULT_CONFIG_MODE = CONFIG_MODE_AHRS;

    // Centralized accessor - easy to change to AP_Param later
    ConfigMode get_config_mode() const {
        return DEFAULT_CONFIG_MODE;  // Future: return _config_mode.get()
    }

    // REVIEW: The three following member variables are accessed both inside the update thread and outside it.
    // Maybe make them atomic to avoid problems due to concurrent access, or introduce a separate
    // HAL_Semaphore to protect them?

    // ArduPilot integration
    bool setup_complete = false;
    uint32_t last_valid_packet_ms = 0;
    
    // Statistics
    uint32_t valid_packets = 0;

    // Core ArduPilot integration methods
    void update_thread();

    // REVIEW: Document what the boolean return means.
    // REVIEW: And maybe it should only return true if there are more bytes waiting to
    // be read, since that's the only case when it makes sense to call it again immediately?
    bool check_uart();
};

#endif  // AP_EXTERNAL_AHRS_SENSAITION_ENABLED