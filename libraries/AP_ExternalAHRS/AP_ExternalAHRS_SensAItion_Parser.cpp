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

#include <stdio.h>
#include <string.h> // Required for memchr, memmove
#include "AP_ExternalAHRS_SensAItion_Parser.h"
#include <AP_HAL/AP_HAL.h>

// Constructor
AP_ExternalAHRS_SensAItion_Parser::AP_ExternalAHRS_SensAItion_Parser(ConfigMode mode) :
    config_mode(mode)
{
    reset_parser();
}

void AP_ExternalAHRS_SensAItion_Parser::reset_parser()
{
    parse_state = ParseState::WAITING_HEADER;
    packet_buffer_len = 0;
    target_payload_len = 0;
    current_packet_id = PacketID::UNKNOWN;
}

// Main parsing loop
void AP_ExternalAHRS_SensAItion_Parser::parse_bytes(const uint8_t* data, size_t data_size, Measurement& measurement)
{
    measurement.type = MeasurementType::UNINITIALIZED;
    
    for (size_t i = 0; i < data_size; i++) {
        if (parse_single_byte(data[i])) {
            // Valid packet found and verified
            decode_packet(measurement);
            
            // Prepare for next packet
            // Note: We reset completely here because we consumed the full buffer successfully.
            reset_parser();
        }
    }
}

// Error Handler
void AP_ExternalAHRS_SensAItion_Parser::handle_invalid_packet()
{
    // Look for a new header byte starting from index 1 
    uint8_t *p = (uint8_t *)memchr(&packet_buffer[1], HEADER_BYTE, packet_buffer_len - 1);

    if (p) {
        // Found a potential new header within the noise/garbage
        size_t bytes_to_discard = p - packet_buffer;
        size_t bytes_to_keep = packet_buffer_len - bytes_to_discard;

        // Shift valid data to the start of the buffer
        memmove(&packet_buffer[0], p, bytes_to_keep);
        packet_buffer_len = bytes_to_keep;

        // Determine State based on Mode and remaining data
        if (config_mode == ConfigMode::INTERLEAVED_INS) {
            // We have the Header (Index 0). Do we have the ID (Index 1)?
            if (packet_buffer_len >= 2) {
                // Technically we have the ID, but the state machine expects to process it via parse_single_byte.
                // However, since we are inside recovery, we can manually check or just wait for more data.
                // For robustness, let's treat it as WAITING_ID.
                // If the ID is invalid, the next cycle will catch it or we verify it now.
                // Simpler: Set to WAITING_ID. The loop logic isn't re-entrant for existing bytes, 
                // so we rely on the parser to just be in a state expecting "more payload" 
                // or if we have enough, we need to handle it. 
                
                // Edge Case: If we shifted so we have Header + ID, we need to process the ID immediately 
                // to set target_len.
                uint8_t id = packet_buffer[1];
                switch (static_cast<PacketID>(id)) {
                    case PacketID::IMU:  target_payload_len = PAYLOAD_SIZE_IMU; parse_state = ParseState::COLLECTING_PAYLOAD; current_packet_id = PacketID::IMU; break;
                    case PacketID::AHRS: target_payload_len = PAYLOAD_SIZE_QUAT; parse_state = ParseState::COLLECTING_PAYLOAD; current_packet_id = PacketID::AHRS; break;
                    case PacketID::INS:  target_payload_len = PAYLOAD_SIZE_INS; parse_state = ParseState::COLLECTING_PAYLOAD; current_packet_id = PacketID::INS; break;
                    default: 
                        // The "New" ID is also bad. Recursively handle or just drop.
                        // Dropping to avoid infinite recursion risk in simple implementations.
                        reset_parser(); 
                        return;
                }
            } else {
                parse_state = ParseState::WAITING_ID;
            }
        } else {
            // Legacy Mode
            target_payload_len = PAYLOAD_SIZE_IMU;
            parse_state = ParseState::COLLECTING_PAYLOAD;
        }
    } else {
        // No header found, reset completely
        reset_parser();
    }
}

// Checksum Validator
bool AP_ExternalAHRS_SensAItion_Parser::buffer_contains_valid_packet() const
{
    // Basic sanity checks
    if (packet_buffer_len < 2 || packet_buffer[0] != HEADER_BYTE) {
        return false;
    }

    // Checksum Logic: XOR everything except Header (Index 0) and Checksum (Last Byte)
    // Legacy: XOR Payload
    // Interleaved: XOR ID + Payload
    // This logic works for both because packet_buffer contains [Header, (ID), Payload..., CRC]
    
    // Validate checksum
    uint8_t calculated = calculate_xor_checksum(packet_buffer, packet_buffer_len - 1);
    uint8_t received = packet_buffer[packet_buffer_len - 1];

    return (calculated == received);
}

// The Core State Machine
bool AP_ExternalAHRS_SensAItion_Parser::parse_single_byte(uint8_t byte)
{
    // Safety: Prevent buffer overflow
    if (packet_buffer_len >= MAX_PACKET_SIZE) {
        handle_invalid_packet();
        // If buffer is still full after recovery (unlikely but possible), hard reset
        if (packet_buffer_len >= MAX_PACKET_SIZE) {
            reset_parser();
        }
    }

    switch (parse_state) {
    case ParseState::WAITING_HEADER:
        if (byte == HEADER_BYTE) {
            packet_buffer_len = 0;
            packet_buffer[packet_buffer_len++] = byte; // Store Header
            
            if (config_mode == ConfigMode::INTERLEAVED_INS) {
                parse_state = ParseState::WAITING_ID;
            } else {
                target_payload_len = PAYLOAD_SIZE_IMU; 
                parse_state = ParseState::COLLECTING_PAYLOAD;
            }
        }
        break;

    case ParseState::WAITING_ID:
        packet_buffer[packet_buffer_len++] = byte; // Store ID
        
        switch (static_cast<PacketID>(byte)) {
        case PacketID::IMU:
            target_payload_len = PAYLOAD_SIZE_IMU; 
            current_packet_id = PacketID::IMU;
            parse_state = ParseState::COLLECTING_PAYLOAD;
            break;
            
        case PacketID::AHRS:
            target_payload_len = PAYLOAD_SIZE_QUAT; 
            current_packet_id = PacketID::AHRS;
            parse_state = ParseState::COLLECTING_PAYLOAD;
            break;
            
        case PacketID::INS:
            target_payload_len = PAYLOAD_SIZE_INS; 
            current_packet_id = PacketID::INS;
            parse_state = ParseState::COLLECTING_PAYLOAD;
            break;
            
        default:
            // Invalid ID -> Trigger recovery immediately
            parse_errors++; // <--- [FIX 2] ADD THIS LINE HERE
            handle_invalid_packet();
            break;
        }
        break;

    case ParseState::COLLECTING_PAYLOAD:
        packet_buffer[packet_buffer_len++] = byte;
        
        // Calculate Expected Total Length
        // Legacy: Header(1) + Payload(N) + CRC(1)
        // Interleaved: Header(1) + ID(1) + Payload(N) + CRC(1)
        size_t overhead = (config_mode == ConfigMode::INTERLEAVED_INS) ? 3 : 2;
        size_t expected_total_len = target_payload_len + overhead;

// ... inside case ParseState::COLLECTING_PAYLOAD ...
        if (packet_buffer_len >= expected_total_len) {
            if (buffer_contains_valid_packet()) {
                valid_packets++;
                return true; // Success!
            } else {
                parse_errors++; // <--- [FIX 1] ADD THIS LINE HERE
                handle_invalid_packet(); // Checksum fail -> Recover
                return false; 
            }
        }
        break;
    }

    return false;
}

bool AP_ExternalAHRS_SensAItion_Parser::validate_checksum() const
{
    return buffer_contains_valid_packet();
}

uint8_t AP_ExternalAHRS_SensAItion_Parser::calculate_xor_checksum(const uint8_t* data, size_t len) const
{
    uint8_t checksum = 0;
    // XOR from Index 1 (skipping Header) up to len (excluding CRC position)
    for (size_t i = 1; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Router
void AP_ExternalAHRS_SensAItion_Parser::decode_packet(Measurement& measurement)
{
    const uint8_t* payload = nullptr;
    
    if (config_mode == ConfigMode::INTERLEAVED_INS) {
        payload = &packet_buffer[2]; // Skip Header + ID
        
        switch (current_packet_id) {
        case PacketID::IMU:
            decode_imu(payload, measurement);
            break;
        case PacketID::AHRS:
            decode_ahrs(payload, measurement);
            break;
        case PacketID::INS:
            decode_ins(payload, measurement);
            break;
        default:
            break;
        }
    } else {
        payload = &packet_buffer[1]; // Skip Header
        decode_imu(payload, measurement);
    }
}
// IMU Decoder (Packet 0)
void AP_ExternalAHRS_SensAItion_Parser::decode_imu(const uint8_t* payload, Measurement& measurement)
{
    // Accel (Bytes 0-11): 3 x Int32 (ug)
    int32_t accel_x_ug = (int32_t)((payload[0]<<24)|(payload[1]<<16)|(payload[2]<<8)|payload[3]);
    int32_t accel_y_ug = (int32_t)((payload[4]<<24)|(payload[5]<<16)|(payload[6]<<8)|payload[7]);
    int32_t accel_z_ug = (int32_t)((payload[8]<<24)|(payload[9]<<16)|(payload[10]<<8)|payload[11]);

    // Gyro (Bytes 12-23): 3 x Int32 (udeg/s)
    int32_t gyro_x_udegs = (int32_t)((payload[12]<<24)|(payload[13]<<16)|(payload[14]<<8)|payload[15]);
    int32_t gyro_y_udegs = (int32_t)((payload[16]<<24)|(payload[17]<<16)|(payload[18]<<8)|payload[19]);
    int32_t gyro_z_udegs = (int32_t)((payload[20]<<24)|(payload[21]<<16)|(payload[22]<<8)|payload[23]);

    // Temp (Bytes 24-25): 1 x Int16 (Scaled)
    int16_t temp_raw = (int16_t)((payload[24]<<8)|payload[25]);

    // Mag (Bytes 26-31): 3 x Int16 (mGauss)
    int16_t mag_x_mgauss = (int16_t)((payload[26]<<8)|payload[27]);
    int16_t mag_y_mgauss = (int16_t)((payload[28]<<8)|payload[29]);
    int16_t mag_z_mgauss = (int16_t)((payload[30]<<8)|payload[31]);

    // Baro (Bytes 32-35): 1 x Int32 (0.1 Pa)
    int32_t baro_raw = (int32_t)((payload[32]<<24)|(payload[33]<<16)|(payload[34]<<8)|payload[35]);

    // Accel: ug -> m/s^2 (Note: 1,000,000 ug = 9.81 m/s^2 roughly)
    const float ug_to_mss = 1.0e-6f * GRAVITY_MSS;
    measurement.acceleration_mss = Vector3f(accel_x_ug, accel_y_ug, accel_z_ug) * ug_to_mss;

    // Gyro: udeg/s -> rad/s (Note: 1,000,000 udeg/s = 1 deg/s = 0.017 rad/s)
    const float udeg_to_rad = 1.0e-6f * DEG_TO_RAD;
    measurement.angular_velocity_rads = Vector3f(gyro_x_udegs, gyro_y_udegs, gyro_z_udegs) * udeg_to_rad;

    // Temp: (Raw * 0.008) + 20
    measurement.temperature_degc = ((float)temp_raw * 0.008f) + 20.0f;

    // Mag: mGauss (Pass-through, ArduPilot expects mGauss)
    measurement.magnetic_field_mgauss = Vector3f(mag_x_mgauss, mag_y_mgauss, mag_z_mgauss);

    // Baro: 0.1 Pa -> Pascal
    measurement.air_pressure_p = (float)baro_raw * 0.1f;

    // Metadata
    measurement.type = MeasurementType::IMU;
    measurement.timestamp_us = AP_HAL::micros64();
}

// AHRS Decoder (Packet 1)
void AP_ExternalAHRS_SensAItion_Parser::decode_ahrs(const uint8_t* payload, Measurement& measurement)
{
    // Payload layout: W (0-3), X (4-7), Y (8-11), Z (12-15)
    int32_t quat_w_raw = (int32_t)((payload[0]<<24)|(payload[1]<<16)|(payload[2]<<8)|payload[3]);
    int32_t quat_x_raw = (int32_t)((payload[4]<<24)|(payload[5]<<16)|(payload[6]<<8)|payload[7]);
    int32_t quat_y_raw = (int32_t)((payload[8]<<24)|(payload[9]<<16)|(payload[10]<<8)|payload[11]);
    int32_t quat_z_raw = (int32_t)((payload[12]<<24)|(payload[13]<<16)|(payload[14]<<8)|payload[15]);

    const float scale_factor = 1.0e-6f;
    
    measurement.orientation = Quaternion(
        (float)quat_w_raw * scale_factor,
        (float)quat_x_raw * scale_factor,
        (float)quat_y_raw * scale_factor,
        (float)quat_z_raw * scale_factor
    );

    measurement.type = MeasurementType::AHRS;
    measurement.timestamp_us = AP_HAL::micros64();
}

// INS Decoder (Packet 2)
void AP_ExternalAHRS_SensAItion_Parser::decode_ins(const uint8_t* payload, Measurement& measurement)
{
    // --- 1. PARSE RAW BYTES (Big-Endian per your snippet style) ---
    // Total Packet Size: 50 Bytes (Indices 0 to 49)

    // Bytes 0-3: Num Sats (Val 43) - 4 Bytes
    // Byte 0: GNSS1 Sat Count. Byte 2: GNSS2 Sat Count.
    measurement.num_sats_gnss1 = payload[0]; // Assuming count is stored in byte 0
    measurement.num_sats_gnss2 = payload[2]; // Assuming count is stored in byte 2

    // Bytes 4-7: Error Flags (Val 47) - 4 Bytes
    measurement.error_flags = (uint32_t)((payload[4]<<24)|(payload[5]<<16)|(payload[6]<<8)|payload[7]);

    // Byte 8: Sensor Valid (Val 48) - 1 Byte
    measurement.sensor_valid = payload[8];

    // Bytes 9-12: Latitude (Val 49) - 4 Bytes (Int32 1e-7 deg)
    int32_t lat_raw = (int32_t)((payload[9]<<24)|(payload[10]<<16)|(payload[11]<<8)|payload[12]);

    // Bytes 13-16: Longitude (Val 50) - 4 Bytes (Int32 1e-7 deg)
    int32_t lon_raw = (int32_t)((payload[13]<<24)|(payload[14]<<16)|(payload[15]<<8)|payload[16]);

    // Bytes 17-28: Velocity N, E, D (Val 51, 52, 54) - 12 Bytes (Int32 mm/s)
    int32_t vel_n_mm = (int32_t)((payload[17]<<24)|(payload[18]<<16)|(payload[19]<<8)|payload[20]);
    int32_t vel_e_mm = (int32_t)((payload[21]<<24)|(payload[22]<<16)|(payload[23]<<8)|payload[24]);
    int32_t vel_d_mm = (int32_t)((payload[25]<<24)|(payload[26]<<16)|(payload[27]<<8)|payload[28]);

    // Bytes 29-32: Altitude MSL (Val 53) - 4 Bytes (Int32 mm)
    int32_t alt_raw_mm = (int32_t)((payload[29]<<24)|(payload[30]<<16)|(payload[31]<<8)|payload[32]);

    // Byte 33: Alignment Status (Val 64) - 1 Byte
    measurement.alignment_status = payload[33];

    // Bytes 34-37: Time iTOW (Val 67) - 4 Bytes (UInt32 ms)
    measurement.time_itow = (uint32_t)((payload[34]<<24)|(payload[35]<<16)|(payload[36]<<8)|payload[37]);

    // Bytes 38-41: GNSS Fix (Val 71) - 4 Bytes (2x UInt16)
    // Bytes 38-39: GNSS1 Fix (UInt16). Bytes 40-41: GNSS2 Fix (UInt16).
    uint16_t gnss1_fix_raw = (uint16_t)((payload[38]<<8)|payload[39]);
    uint16_t gnss2_fix_raw = (uint16_t)((payload[40]<<8)|payload[41]);
    
    measurement.gnss1_fix = (uint8_t)gnss1_fix_raw;
    measurement.gnss2_fix = (uint8_t)gnss2_fix_raw;

    // Bytes 42-45: Quality Pos (Val 97) - 4 Bytes (Int32 mm)
    int32_t pos_acc_mm = (int32_t)((payload[42]<<24)|(payload[43]<<16)|(payload[44]<<8)|payload[45]);

    // Bytes 46-49: Quality Vel (Val 99) - 4 Bytes (Int32 mm/s)
    int32_t vel_acc_mm = (int32_t)((payload[46]<<24)|(payload[47]<<16)|(payload[48]<<8)|payload[49]);


    // --- 2. APPLY UNIT CONVERSIONS & POPULATE ---
    measurement.location.lat = lat_raw;
    measurement.location.lng = lon_raw;
    measurement.location.alt = alt_raw_mm / 10; // mm -> cm (ArduPilot uses cm for Alt)

    const float mms_to_ms = 0.001f;
    measurement.velocity_ned = Vector3f(vel_n_mm, vel_e_mm, vel_d_mm) * mms_to_ms;

    measurement.pos_accuracy_horiz = (float)pos_acc_mm * mms_to_ms; // mm -> m
    measurement.pos_accuracy_vert  = (float)pos_acc_mm * mms_to_ms; // mm -> m
    measurement.vel_accuracy       = (float)vel_acc_mm * mms_to_ms; // mm/s -> m/s

    measurement.type = MeasurementType::INS;
    measurement.timestamp_us = AP_HAL::micros64();
}