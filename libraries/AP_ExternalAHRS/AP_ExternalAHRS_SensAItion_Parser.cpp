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
  SensAItion Protocol Parser Implementation
  Pure parsing logic - no ArduPilot dependencies
*/

#include "AP_ExternalAHRS_SensAItion_Parser.h"

// Constructor
AP_ExternalAHRS_SensAItion_Parser::AP_ExternalAHRS_SensAItion_Parser(ConfigMode mode) :
    config_mode(mode),
    parse_state(ParseState::LOOKING_FOR_HEADER),
    packet_buffer_len(0),
    valid_packets(0),
    parse_errors(0)
{
}

// Reset parser to initial state
void AP_ExternalAHRS_SensAItion_Parser::reset()
{
    parse_state = ParseState::LOOKING_FOR_HEADER;
    packet_buffer_len = 0;
    valid_packets = 0;
    parse_errors = 0;
}

// Parse multiple bytes from UART stream
bool AP_ExternalAHRS_SensAItion_Parser::parse_bytes(const uint8_t* data, size_t data_size, const uint8_t*& packet_out, size_t& packet_size_out)
{
    for (size_t i = 0; i < data_size; i++) {
        if (parse_single_byte(data[i], packet_out, packet_size_out)) {
            return true;  // Found a complete packet
        }
    }
    return false;  // No complete packet yet
}

// Parse a single byte through state machine
bool AP_ExternalAHRS_SensAItion_Parser::parse_single_byte(uint8_t byte, const uint8_t*& packet_out, size_t& packet_size_out)
{
    switch (parse_state) {
    case ParseState::LOOKING_FOR_HEADER:
        if (byte == HEADER_BYTE) {
            packet_buffer_len = 0;
            packet_buffer[packet_buffer_len++] = byte;
            parse_state = ParseState::COLLECTING_PACKET;
        }
        break;

    case ParseState::COLLECTING_PACKET:
        packet_buffer[packet_buffer_len++] = byte;

        size_t expected_size = get_expected_packet_size();

        if (packet_buffer_len == expected_size) {
            // Validate complete packet
            if (validate_packet(packet_buffer, packet_buffer_len)) {
                valid_packets++;
                // Return pointer to data portion (skip header byte)
                packet_out = packet_buffer + 1;
                // Return data size (exclude header and checksum)
                packet_size_out = packet_buffer_len - 2;
                parse_state = ParseState::LOOKING_FOR_HEADER;
                return true;  // Complete valid packet
            } else {
                parse_errors++;
                parse_state = ParseState::LOOKING_FOR_HEADER;
            }
        } else if (packet_buffer_len >= MAX_PACKET_SIZE) {
            // Prevent buffer overflow
            parse_errors++;
            parse_state = ParseState::LOOKING_FOR_HEADER;
        }
        break;
    }

    return false;
}

// Validate a complete packet (size and checksum)
bool AP_ExternalAHRS_SensAItion_Parser::validate_packet(const uint8_t* packet, size_t packet_size)
{
    size_t expected_size = get_expected_packet_size();

    // Validate packet structure
    if (packet_size != expected_size || packet[0] != HEADER_BYTE) {
        return false;
    }

    // Validate checksum
    uint8_t calculated = calculate_xor_checksum(packet, 1, packet_size - 2);
    uint8_t received = packet[packet_size - 1];

    return (calculated == received);
}

// Calculate XOR checksum
uint8_t AP_ExternalAHRS_SensAItion_Parser::calculate_xor_checksum(const uint8_t* data, size_t start, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = start; i < start + length && i < MAX_PACKET_SIZE; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
