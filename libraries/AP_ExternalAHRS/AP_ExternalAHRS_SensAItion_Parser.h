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
  SensAItion Protocol Parser
  Handles binary protocol parsing (state machine, packet validation, checksums)
  Separated from ArduPilot state management for testability
*/

#pragma once

#include <stdint.h>
#include <stddef.h>

class AP_ExternalAHRS_SensAItion_Parser
{
public:
    // Configuration modes
    enum class ConfigMode {
        CONFIG_MODE_IMU = 0,   // IMU only mode (accel, gyro, mag, baro, temp)
        CONFIG_MODE_AHRS = 1   // AHRS mode (IMU + quaternion)
    };

    // Constructor
    AP_ExternalAHRS_SensAItion_Parser(ConfigMode mode = ConfigMode::CONFIG_MODE_IMU);

    // Parse multiple bytes from UART stream
    // When a complete packet is validated, packet_out will point to the buffer and packet_size_out will be set
    // Returns: true if a complete, validated packet was found
    bool parse_bytes(const uint8_t* data, size_t data_size, const uint8_t*& packet_out, size_t& packet_size_out);

    // Get parser statistics
    uint32_t get_valid_packets() const
    {
        return valid_packets;
    }
    uint32_t get_parse_errors() const
    {
        return parse_errors;
    }

    // Reset parser state
    void reset();

    // Protocol constants (public for driver to use)
    static constexpr uint8_t HEADER_BYTE = 0xFA;
    static constexpr int MAX_PACKET_SIZE = 256;
    static constexpr size_t PACKET_SIZE_IMU = 38;   // 36 data bytes + header + checksum
    static constexpr size_t PACKET_SIZE_AHRS = 54;  // 52 data bytes + header + checksum

private:
    // Packet parsing state machine
    enum class ParseState {
        LOOKING_FOR_HEADER,
        COLLECTING_PACKET
    };

    // Member variables
    ConfigMode config_mode;
    ParseState parse_state;
    uint8_t packet_buffer[MAX_PACKET_SIZE];
    uint16_t packet_buffer_len;

    // Statistics
    uint32_t valid_packets;
    uint32_t parse_errors;

    // Internal parsing methods
    bool parse_single_byte(uint8_t byte, const uint8_t*& packet_out, size_t& packet_size_out);
    bool validate_packet(const uint8_t* packet, size_t packet_size);
    uint8_t calculate_xor_checksum(const uint8_t* data, size_t start, size_t length);

    // Get expected packet size for current mode
    size_t get_expected_packet_size() const
    {
        return (config_mode == ConfigMode::CONFIG_MODE_IMU) ? PACKET_SIZE_IMU : PACKET_SIZE_AHRS;
    }
};
