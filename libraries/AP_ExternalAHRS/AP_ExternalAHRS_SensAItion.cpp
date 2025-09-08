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
  Support for SensAItion serial connected AHRS and IMU
  Implements SensAItion protocol with ArduPilot-specific adaptations
*/

#include "AP_ExternalAHRS_SensAItion.h"

#if AP_EXTERNAL_AHRS_SENSAITION_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_SensAItion::AP_ExternalAHRS_SensAItion(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state),
    parse_state_(LOOKING_FOR_HEADER),
    packet_buffer_len_(0),
    uart(nullptr),
    baudrate(460800),  
    port_num(-1),
    imu_candidate("IMU"),
    ahrs_candidate("AHRS"),
    active_config(nullptr),
    configs_initialized(false),
    setup_complete(false),
    last_valid_packet_ms(0),
    complete_packets(0),
    valid_packets(0),
    invalid_checksums(0)
{
    // Initialize sensor definitions according to SensAItion protocol specification
    initializeSensorDefinitions();
    
    // Allocate packet buffer
    packet_buffer_ = NEW_NOTHROW uint8_t[MAX_PACKET_SIZE];
    if (!packet_buffer_) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SensAItion: Failed to allocate packet buffer");
        return;
    }
    
    // ArduPilot-specific: Initialize automatic config detection system (remove detection system and implemente Mission Planner config later)
    initialize();
    
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    
    // Create thread for non-blocking UART processing
    if (!hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SensAItion::update_thread, void),
        "AHRS_SensAItion", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("SensAItion Failed to start ExternalAHRS update thread");
    }
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion ExternalAHRS initialised");
}

// Standard SensAItion parser implementation - initializes all sensor definitions
void AP_ExternalAHRS_SensAItion::initializeSensorDefinitions() {
    // Initialize all sensor definitions based on SensAItion manual Appendix A
    
    // Helper function to define a sensor
    auto defineSensor = [this](int id, const char* name, const char* unit, DataType type, double scale) {
        sensor_definitions_[id].name = name;
        sensor_definitions_[id].unit = unit;
        sensor_definitions_[id].type = type;
        sensor_definitions_[id].scale_factor = scale;
        sensor_definitions_[id].valid = false;
        sensor_definitions_[id].timestamp_ms = 0;
    };
    
    // IMU sensors
    defineSensor(0x00, "accel_x", "g", DataType::FLOAT, 1e-6);
    defineSensor(0x01, "accel_y", "g", DataType::FLOAT, 1e-6);
    defineSensor(0x02, "accel_z", "g", DataType::FLOAT, 1e-6);
    defineSensor(0x03, "gyro_x", "deg/s", DataType::FLOAT, 1e-6);
    defineSensor(0x04, "gyro_y", "deg/s", DataType::FLOAT, 1e-6);
    defineSensor(0x05, "gyro_z", "deg/s", DataType::FLOAT, 1e-6);
    
    // Inclinometer
    defineSensor(0x06, "incl_x", "g", DataType::FLOAT, 1e-6);
    defineSensor(0x07, "incl_y", "g", DataType::FLOAT, 1e-6);
    defineSensor(0x08, "incl_z", "g", DataType::FLOAT, 1e-6);
    
    // Temperature and magnetometer  
    defineSensor(0x09, "imu_temp", "°C", DataType::FLOAT, 0.01);
    defineSensor(0x0A, "mag_x", "mGauss", DataType::INT32, 1.0);
    defineSensor(0x0B, "mag_y", "mGauss", DataType::INT32, 1.0);
    defineSensor(0x0C, "mag_z", "mGauss", DataType::INT32, 1.0);
    defineSensor(0x0D, "barometer", "hPa", DataType::FLOAT, 0.1);
    defineSensor(0x0E, "odometer_speed", "m/s", DataType::FLOAT, 1e-3);
    defineSensor(0x0F, "cal_imu_temp", "°C", DataType::FLOAT, 0.01);
    
    // Position data removed - SensAItion supports only AHRS and IMU
    
    // Attitude data (AHRS mode)  
    defineSensor(0x37, "roll", "deg", DataType::FLOAT, 1e-6);
    defineSensor(0x38, "pitch", "deg", DataType::FLOAT, 1e-6);
    defineSensor(0x39, "yaw", "deg", DataType::FLOAT, 1e-6);
    
    // Quaternion data (AHRS mode) - from user manual
    defineSensor(0x4C, "quat_w", "", DataType::FLOAT, 1e-6);  // Quaternion scalar
    defineSensor(0x4D, "quat_x", "", DataType::FLOAT, 1e-6);  // Quaternion i
    defineSensor(0x4E, "quat_y", "", DataType::FLOAT, 1e-6);  // Quaternion j
    defineSensor(0x4F, "quat_z", "", DataType::FLOAT, 1e-6);  // Quaternion k
    
    // Time and status
    defineSensor(0x42, "time_tick", "ms", DataType::UINT32, 1.0);
    defineSensor(0x2F, "error_flags", "", DataType::FLAGS32, 1.0);
    defineSensor(0x40, "attitude_status", "", DataType::FLAGS16, 1.0);
    defineSensor(0x41, "alignment_status", "", DataType::FLAGS16, 1.0);
    
    // Quality values
    defineSensor(0x67, "quality_roll", "deg", DataType::FLOAT, 1e-6);
    defineSensor(0x68, "quality_pitch", "deg", DataType::FLOAT, 1e-6);
    defineSensor(0x69, "quality_yaw", "deg", DataType::FLOAT, 1e-6);
    defineSensor(0x61, "quality_pos_north", "m", DataType::FLOAT, 1e-3);
    defineSensor(0x62, "quality_pos_east", "m", DataType::FLOAT, 1e-3);
    defineSensor(0x65, "quality_pos_down", "m", DataType::FLOAT, 1e-3);
    defineSensor(0x63, "quality_vel_north", "m/s", DataType::FLOAT, 1e-3);
    defineSensor(0x64, "quality_vel_east", "m/s", DataType::FLOAT, 1e-3);
    defineSensor(0x66, "quality_vel_down", "m/s", DataType::FLOAT, 1e-3);
    
    // GNSS data
    defineSensor(0x43, "gnss_num_satellites", "", DataType::UINT8x4, 1.0);
    defineSensor(0x4A, "gnss_fix_type", "", DataType::UINT16x2, 1.0);

}

// Standard SensAItion parser implementation, with ArduPilot debug messaging
bool AP_ExternalAHRS_SensAItion::parseConfigString(const char* config) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: parseConfigString called with: %.50s...", config);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: parseConfigString BEFORE - num_bytes=%d", config_.num_bytes);
    
    config_.data_layout_count = 0;
    config_.num_bytes = 0;
    config_.data_rate = 1;
    config_.using_checksum = false;
    config_.using_big_endian = true;
    
    size_t pos = 0;
    size_t len = strlen(config);
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Config string length: %u", (unsigned)len);
    
    // Verify 's' value in config string
    const char* s_pos = strchr(config, 's');
    if (s_pos && s_pos[1] && s_pos[2]) {
        char s_value[3] = {s_pos[1], s_pos[2], 0};
        int s_int = strtol(s_value, NULL, 16);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Config 's' value: %s (hex) = %d (decimal)", s_value, s_int);
    }
    
    while (pos < len) {
        char identifier = config[pos++];
        
        switch (identifier) {
        case 'o': {
            // Data rate: oXXXX
            if (pos + 4 > len) return false;
            uint16_t rate_divisor = 0;
            for (int i = 0; i < 4; i++) {
                rate_divisor = (rate_divisor << 4) | charToHex(config[pos++]);
            }
            config_.data_rate = rate_divisor;
            break;
        }
        
        case 's': {
            // Sensor data: sXXYYYZZZ... where XX=length, YYY=sensor, Z=byte
            if (pos + 2 > len) return false;
            uint8_t hex1 = charToHex(config[pos++]);
            uint8_t hex2 = charToHex(config[pos++]);
            uint8_t num_bytes = (hex1 << 4) | hex2;
            
            for (int i = 0; i < num_bytes; i++) {
                if (pos + 3 > len || config_.data_layout_count >= ParseConfig::MAX_DATA_ENTRIES) break;
                
                ParseConfig::DataEntry entry;
                uint8_t sensor_hex1 = charToHex(config[pos++]);
                uint8_t sensor_hex2 = charToHex(config[pos++]);
                entry.sensor_id = (sensor_hex1 << 4) | sensor_hex2;
                entry.byte_index = charToHex(config[pos++]);
                config_.data_layout[config_.data_layout_count++] = entry;
                config_.num_bytes++;
            }
            break;
        }
        
        case 'x':
            // Checksum enabled
            config_.using_checksum = true;
            config_.num_bytes++; // Account for checksum byte
            break;
            
        default:
            // Unknown identifier, skip
            break;
        }
    }
    
    configs_initialized = true;
    
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: parseConfigString SUCCESS - num_bytes=%d, data_layout_count=%d", 
                  config_.num_bytes, config_.data_layout_count);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Config - checksum=%s, big_endian=%s", 
                  config_.using_checksum ? "ON" : "OFF", config_.using_big_endian ? "TRUE" : "FALSE");
    
    return true;
}

// Standard SensAItion parser implementation with ArduPilot config detection added, ardupilot debug messaging added
bool AP_ExternalAHRS_SensAItion::parseMultipleBytes(const uint8_t* data, size_t data_size) {
    bool processed_any = false;
    
    for (size_t i = 0; i < data_size; i++) {
        uint8_t byte = data[i];
        
        switch (parse_state_) {
        case LOOKING_FOR_HEADER:
            if (byte == HEADER_BYTE) {
                packet_buffer_len_ = 0;
                packet_buffer_[packet_buffer_len_++] = byte;
                parse_state_ = COLLECTING_PACKET;
            }
            break;
            
        case COLLECTING_PACKET:
            packet_buffer_[packet_buffer_len_++] = byte;
            
            // ArduPilot-specific: Dynamic packet size detection
            // Validates against both IMU and AHRS configurations simultaneously
            if (!active_config) {
                // Detection phase - test both candidates if packet could match
                bool packet_complete = false;
                
                // Check if packet size matches either candidate expectation
                size_t imu_expected = imu_candidate.config.num_bytes + 1;
                size_t ahrs_expected = ahrs_candidate.config.num_bytes + 1; 
                
                if (packet_buffer_len_ == imu_expected) {
                    // Test IMU candidate
                    if (validateConfigCandidate(imu_candidate, packet_buffer_, packet_buffer_len_)) {
                        updateCandidateSuccess(imu_candidate);
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: IMU validation success (%u/%u)", 
                                      imu_candidate.consecutive_successes, LOCK_THRESHOLD);
                        packet_complete = true;  // Only complete if validation succeeds
                    } else {
                        updateCandidateFailure(imu_candidate);
                        // Don't set packet_complete - keep collecting for AHRS test
                    }
                }
                
                if (packet_buffer_len_ == ahrs_expected) {
                    // Test AHRS candidate  
                    if (validateConfigCandidate(ahrs_candidate, packet_buffer_, packet_buffer_len_)) {
                        updateCandidateSuccess(ahrs_candidate);
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: AHRS validation success (%u/%u)", 
                                      ahrs_candidate.consecutive_successes, LOCK_THRESHOLD);
                        packet_complete = true;  // Only complete if validation succeeds
                    } else {
                        updateCandidateFailure(ahrs_candidate);
                        packet_complete = true;  // AHRS is largest - must complete here
                    }
                }
                
                // Safety: If we exceed max expected size without success, reset
                if (packet_buffer_len_ > ahrs_expected) {
                    packet_complete = true;
                }
                
                // Check if we should switch to an active config
                if (shouldSwitchConfig()) {
                    // Thread safety: Protect config changes with semaphore
                    WITH_SEMAPHORE(state.sem);
                    
                    if (imu_candidate.consecutive_successes >= LOCK_THRESHOLD) {
                        active_config = &imu_candidate;
                        config_ = imu_candidate.config;
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Locked to IMU config (%d bytes)", 
                                      config_.num_bytes);
                    } else if (ahrs_candidate.consecutive_successes >= LOCK_THRESHOLD) {
                        active_config = &ahrs_candidate;
                        config_ = ahrs_candidate.config;
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Locked to AHRS config (%d bytes)", 
                                      config_.num_bytes);
                    }
                }
                
                if (packet_complete) {
                    // Parse packet during detection phase using matching config
                    ParseConfig original_config = config_;  // Save current config
                    bool config_set = false;
                    
                    if (packet_buffer_len_ == imu_expected) {
                        config_ = imu_candidate.config;
                        config_set = true;
                    } else if (packet_buffer_len_ == ahrs_expected) {
                        config_ = ahrs_candidate.config;
                        config_set = true;
                    }
                    
                    // Only parse if we have a valid configuration match
                    if (config_set && parsePacket(packet_buffer_, packet_buffer_len_)) {
                        processed_any = true;
                        // Sensor data is now available for ArduPilot
                    } else if (!config_set) {
                        static uint32_t unknown_size_debug = 0;
                        if (++unknown_size_debug % 1000 == 0) {
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: Unknown packet size %u bytes", 
                                          (unsigned)packet_buffer_len_);
                        }
                    }
                    
                    // Restore config if we haven't locked to a specific one yet
                    if (!active_config) {
                        config_ = original_config;
                    }
                    
                    parse_state_ = LOOKING_FOR_HEADER;
                }
                
            } else {
                // Active config mode - use standard SensAItionParser logic, ardupilot debug messages added
                if (config_.num_bytes > 0 && 
                    packet_buffer_len_ >= static_cast<size_t>(config_.num_bytes + 1)) {
                    
                    bool parse_success = parsePacket(packet_buffer_, packet_buffer_len_);
                    if (parse_success) {
                        updateCandidateSuccess(*active_config);
                        processed_any = true;
                    } else {
                        updateCandidateFailure(*active_config);
                        // Check if we should unlock due to failures
                        if (shouldSwitchConfig() && active_config->consecutive_failures >= UNLOCK_FAILURES) {
                            // Thread safety: Protect config unlock with semaphore
                            WITH_SEMAPHORE(state.sem);
                            
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: Unlocking %s config due to failures", 
                                          active_config->config_name);
                            active_config = nullptr;
                            
                            // Reset to IMU config for re-detection
                            config_ = imu_candidate.config;
                            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Reset to IMU config for re-detection");
                        }
                    }
                    parse_state_ = LOOKING_FOR_HEADER;
                }
            }
            
            if (packet_buffer_len_ > MAX_PACKET_SIZE) {
                // Prevent buffer overflow
                parse_state_ = LOOKING_FOR_HEADER;
            }
            break;
        }
    }
    
    return processed_any;
}

// Standard SensAItion parser implementation - packet validation and parsing, with ardupilot debugging added
bool AP_ExternalAHRS_SensAItion::parsePacket(const uint8_t* packet, size_t packet_size) {
    static uint32_t parse_debug = 0;
    bool debug_this = (++parse_debug % 1000 == 0);  // Debug every 1000th packet
    
    if (debug_this) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: parsePacket called - size=%u, config_bytes=%d",
                      (unsigned)packet_size, config_.num_bytes);
    }
    
    // Validate packet
    if (packet_size == 0 || packet[0] != HEADER_BYTE) {
        return false;
    }
    
    // Validate configuration parameters
    if (config_.num_bytes <= 0 || config_.num_bytes > MAX_PACKET_SIZE) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SensAItion: Invalid config num_bytes=%d", config_.num_bytes);
        return false;
    }
    
    if (config_.data_layout_count == 0 || config_.data_layout_count > ParseConfig::MAX_DATA_ENTRIES) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SensAItion: Invalid data_layout_count=%u", config_.data_layout_count);
        return false;
    }
    
    // Checksum byte already counted in config_.num_bytes
    size_t expected_size = config_.num_bytes + 1; // +1 for header (checksum already counted)
    
    bool size_ok = (packet_size == expected_size);
    if (size_ok) {
        static uint32_t size_ok_debug = 0;
        if (++size_ok_debug % 20000 == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Size OK - %u bytes (checksum=%s)", 
                          (unsigned)packet_size, config_.using_checksum ? "yes" : "no");
        }
    } else {
        // Try to accept packet anyway for testing
        size_ok = true;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: Size mismatch - got %u, expected %u, trying anyway", 
                      (unsigned)packet_size, (unsigned)expected_size);
    }
    
    if (!size_ok) {
        return false;
    }
    
    // Validate checksum if enabled
    if (config_.using_checksum) {
        uint8_t calculated = calculateXorChecksum(packet, 1, packet_size - 2);
        uint8_t received = packet[packet_size - 1];
        if (calculated != received) {
            invalid_checksums++;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: Checksum FAIL - calc=0x%02X recv=0x%02X (fail #%u)",
                          calculated, received, (unsigned)invalid_checksums);
            return false;
        } else {
            static uint32_t checksum_ok_debug = 0;
            if (++checksum_ok_debug % 40000 == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Checksum OK - calc=0x%02X recv=0x%02X",
                              calculated, received);
            }
        }
    }
    
    if (debug_this) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: parsePacket - checksum OK, calling extractSensorData");
    }
    
    // Extract sensor data
    bool extract_result = extractSensorData(packet);
    
    if (debug_this) {
        if (extract_result) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: parsePacket SUCCESS - packet parsed OK");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: parsePacket FAIL - extractSensorData returned false");
        }
    }
    
    return extract_result;
}

// Standard SensAItion parser implementation, with debug messages added
bool AP_ExternalAHRS_SensAItion::extractSensorData(const uint8_t* packet) {
    WITH_SEMAPHORE(state.sem);
    
    // Clear previous sensor data validity
    for (int i = 0; i <= MAX_SENSOR_ID; i++) {
        sensor_values_[i].valid = false;
    }
    
    // Group bytes by sensor ID using static arrays
    static int sensor_byte_indices[256][4]; // Max 4 bytes per sensor
    static int sensor_byte_counts[256];
    
    // Reset counts
    for (int i = 0; i < 256; i++) {
        sensor_byte_counts[i] = 0;
    }
    
    // Group bytes by sensor ID
    for (int i = 0; i < config_.data_layout_count; i++) {
        const auto& entry = config_.data_layout[i];
        if (sensor_byte_counts[entry.sensor_id] < 4) {
            sensor_byte_indices[entry.sensor_id][sensor_byte_counts[entry.sensor_id]] = i + 1; // +1 to skip header
            sensor_byte_counts[entry.sensor_id]++;
        }
    }
    
    // Process each sensor - debug which sensors found
    uint8_t sensors_found = 0;
    
    for (int sensor_id = 0; sensor_id <= MAX_SENSOR_ID; sensor_id++) {
        if (sensor_byte_counts[sensor_id] == 0) continue;
        sensors_found++;
        
        // Combine bytes into 32-bit value
        int32_t raw_value = combineBytes(packet, sensor_byte_indices[sensor_id], sensor_byte_counts[sensor_id], config_.using_big_endian);
        updateSensorValue(sensor_id, raw_value);
        
        if (sensors_found <= 8) {  // Show first 8 sensors
            static uint32_t sensor_debug = 0;
            if (++sensor_debug % 4000 == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Sensor 0x%02X = raw %ld (%d bytes)",
                              sensor_id, (long)raw_value, sensor_byte_counts[sensor_id]);
            }
        }
    }
    
    static uint32_t extract_summary_debug = 0;
    if (++extract_summary_debug % 2000 == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: extractSensorData - found %u sensors", sensors_found);
    }
    
    if (sensors_found == 0) {
        static uint32_t no_sensors_debug = 0;
        if (++no_sensors_debug % 1000 == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: extractSensorData - NO SENSORS FOUND!");
        }
        return false;  // No sensors processed
    }
    
    return true;
}

// Standard SensAItion parser implementation
int32_t AP_ExternalAHRS_SensAItion::combineBytes(const uint8_t* packet, const int* byte_indices, int num_bytes, bool big_endian) {
    int32_t result = 0;
    
    if (big_endian) {
        for (int i = 0; i < num_bytes; i++) {
            int idx = byte_indices[i];
            if (idx >= 0 && idx < MAX_PACKET_SIZE) {
                result = (result << 8) | packet[idx];
            }
        }
    } else {
        for (int i = num_bytes - 1; i >= 0; i--) {
            int idx = byte_indices[i];
            if (idx >= 0 && idx < MAX_PACKET_SIZE) {
                result = (result << 8) | packet[idx];
            }
        }
    }
    
    return result;
}

// Standard SensAItion parser implementation, with ardupiilot debug messages added
void AP_ExternalAHRS_SensAItion::updateSensorValue(int sensor_id, int32_t raw_value) {
    if (sensor_id < 0 || sensor_id > MAX_SENSOR_ID) return;
    
    const auto& definition = sensor_definitions_[sensor_id];
    SensorValue& sensor_value = sensor_values_[sensor_id];
    
    // Copy definition info
    sensor_value.name = definition.name;
    sensor_value.unit = definition.unit;
    sensor_value.type = definition.type;
    sensor_value.scale_factor = definition.scale_factor;
    
    // Convert raw value based on data type
    switch (definition.type) {
    case DataType::FLOAT16:
        raw_value = static_cast<int32_t>(static_cast<int16_t>(raw_value)); // Sign extend
        [[fallthrough]];
    case DataType::FLOAT:
        // Special handling for temperature sensor (0x09) - debug conversion
        if (sensor_id == 0x09) {
            // Temperature conversion: Temp °C = (raw/10000) * 80 + 20
            double temp_celsius = (static_cast<double>(raw_value) / 10000.0) * 80.0 + 20.0;
            sensor_value.data.dfloat = temp_celsius;
            
            static uint32_t temp_debug = 0;
            if (++temp_debug % 10000 == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Temp conversion - raw=%ld -> %.1f°C",
                              (long)raw_value, temp_celsius);
            }
        } else {
            sensor_value.data.dfloat = definition.scale_factor * static_cast<double>(raw_value);
            // Debug quaternion values specifically
            if (sensor_id >= 0x4C && sensor_id <= 0x4F) {
                static uint32_t quat_debug[4] = {0, 0, 0, 0};
                int quat_idx = sensor_id - 0x4C;
                if (++quat_debug[quat_idx] % 100 == 0) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Quat sensor 0x%02X - raw=%ld, scale=%.9f, result=%.6f",
                                  sensor_id, (long)raw_value, definition.scale_factor, sensor_value.data.dfloat);
                }
            }
        }
        break;
        
    case DataType::INT8:
        sensor_value.data.dint = static_cast<int32_t>(static_cast<int8_t>(raw_value));
        break;
        
    case DataType::INT16:
        sensor_value.data.dint = static_cast<int32_t>(static_cast<int16_t>(raw_value));
        break;
        
    case DataType::INT32:
        sensor_value.data.dint = raw_value;
        break;
        
    case DataType::UINT8:
    case DataType::UINT16:
    case DataType::UINT32:
    case DataType::FLAGS8:
    case DataType::FLAGS16:
    case DataType::FLAGS32:
        sensor_value.data.duint = static_cast<uint32_t>(raw_value);
        break;
        
    case DataType::UINT16x2:
        sensor_value.data.duint16x2[0] = static_cast<uint16_t>((static_cast<uint32_t>(raw_value) >> 16) & 0xFFFF);
        sensor_value.data.duint16x2[1] = static_cast<uint16_t>(static_cast<uint32_t>(raw_value) & 0xFFFF);
        break;
        
    case DataType::UINT8x4:
        sensor_value.data.duint8x4[0] = static_cast<uint8_t>((static_cast<uint32_t>(raw_value) >> 24) & 0xFF);
        sensor_value.data.duint8x4[1] = static_cast<uint8_t>((static_cast<uint32_t>(raw_value) >> 16) & 0xFF);
        sensor_value.data.duint8x4[2] = static_cast<uint8_t>((static_cast<uint32_t>(raw_value) >> 8) & 0xFF);
        sensor_value.data.duint8x4[3] = static_cast<uint8_t>(static_cast<uint32_t>(raw_value) & 0xFF);
        break;
    }
    
    sensor_value.valid = true;
    sensor_value.timestamp_ms = AP_HAL::millis();
}

// Data access methods
bool AP_ExternalAHRS_SensAItion::getSensorValue(int sensor_id, double& value) const {
    WITH_SEMAPHORE(state.sem);
    
    if (sensor_id < 0 || sensor_id > MAX_SENSOR_ID || !sensor_values_[sensor_id].valid) {
        return false;
    }
    
    const auto& sensor = sensor_values_[sensor_id];
    switch (sensor.type) {
    case DataType::FLOAT:
    case DataType::FLOAT16:
        value = sensor.data.dfloat;
        break;
    case DataType::INT8:
    case DataType::INT16:
    case DataType::INT32:
        value = static_cast<double>(sensor.data.dint);
        break;
    case DataType::UINT8:
    case DataType::UINT16:
    case DataType::UINT32:
    case DataType::FLAGS8:
    case DataType::FLAGS16:
    case DataType::FLAGS32:
        value = static_cast<double>(sensor.data.duint);
        break;
    case DataType::UINT16x2:
        value = static_cast<double>(sensor.data.duint16x2[0]); // Return first element
        break;
    case DataType::UINT8x4:
        value = static_cast<double>(sensor.data.duint8x4[0]); // Return first element
        break;
    }
    
    return true;
}

bool AP_ExternalAHRS_SensAItion::isSensorValid(int sensor_id) const {
    WITH_SEMAPHORE(state.sem);
    return sensor_id >= 0 && sensor_id <= MAX_SENSOR_ID && sensor_values_[sensor_id].valid;
}

// Utility methods
uint8_t AP_ExternalAHRS_SensAItion::calculateXorChecksum(const uint8_t* data, size_t start, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = start; i < start + length && i < MAX_PACKET_SIZE; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

uint8_t AP_ExternalAHRS_SensAItion::charToHex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0; // Invalid hex character
}

void AP_ExternalAHRS_SensAItion::clearData() {
    for (int i = 0; i <= MAX_SENSOR_ID; i++) {
        sensor_values_[i].valid = false;
    }
}

// ArduPilot interface methods
int8_t AP_ExternalAHRS_SensAItion::get_port() const {
    if (!uart) {
        return -1;
    }
    return port_num;
}

bool AP_ExternalAHRS_SensAItion::healthy() const {
    uint32_t now_ms = AP_HAL::millis();
    return (now_ms - last_valid_packet_ms) < 1000 && valid_packets > 0;
}

bool AP_ExternalAHRS_SensAItion::initialised() const {
    return setup_complete && configs_initialized;
}

bool AP_ExternalAHRS_SensAItion::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const {
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SensAItion unhealthy");
        return false;
    }
    return true;
}

const char* AP_ExternalAHRS_SensAItion::get_name() const {
    return "SensAItion";
}

void AP_ExternalAHRS_SensAItion::get_filter_status(nav_filter_status &status) const {
    status = {};
    if (healthy()) {
        status.flags.attitude = true;
        status.flags.horiz_vel = true;
        status.flags.vert_vel = true;
        status.flags.horiz_pos_rel = false; // Are these needed if we don't implement INS right now?
        status.flags.horiz_pos_abs = false;
        status.flags.vert_pos = false;
        status.flags.using_gps = false;
        status.flags.gps_glitching = false;
        status.flags.initalized = true;
    }
}

bool AP_ExternalAHRS_SensAItion::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const {
    velVar = 0.1f;
    posVar = 0.1f;
    hgtVar = 0.1f;
    magVar.x = magVar.y = magVar.z = 0.01f;
    tasVar = 0.1f;
    return true;
}

// Thread function for non-blocking UART processing
void AP_ExternalAHRS_SensAItion::update_thread() {
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay(1);
        }
    }
}

// Main update method - now empty since thread handles everything
void AP_ExternalAHRS_SensAItion::update() {
    // Thread handles all UART processing
}

bool AP_ExternalAHRS_SensAItion::check_uart() {
    if (!uart) {
        return false;
    }
    
    if (!setup_complete) {
        uart->begin(baudrate);
        setup_complete = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: UART initialized at %u baud on port %d", 
                      (unsigned)baudrate, (int)port_num);
        
        // ArduPilot-specific auto-configuration
        // Unlike the reference parser, we can't get config dynamically so we detect packet size
        if (!configs_initialized) {
            // Initialize config candidates
            initialize();
            // Start with AHRS config as default - will auto-adjust on first packet
            if (parseConfigString(AHRS_CONFIG)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Auto-config initialized, will detect packet size");
                configs_initialized = true;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SensAItion: Failed to parse config string");
            }
        }
    }
    
    // Read available data from UART
    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }
    
    // Read up to 256 bytes at a time
    uint8_t buffer[256];
    n = MIN(n, sizeof(buffer));
    ssize_t nread = uart->read(buffer, n);
    
    if (nread > 0) {
        // Process the data through the parser
        if (parseMultipleBytes(buffer, nread)) {
            // Data was successfully processed - update ArduPilot
            update_ardupilot_imu_data();
            update_ardupilot_attitude_data();
        }
        return true;
    }
    
    return false;
}

// ArduPilot-specific: Initialize automatic config detection system
// Sets up IMU and AHRS configuration candidates for runtime detection
void AP_ExternalAHRS_SensAItion::initialize() {
    // Initialize config candidates
    imu_candidate = ConfigCandidate("IMU");
    ahrs_candidate = ConfigCandidate("AHRS");
    
    // Parse both configurations upfront
    if (!parseConfigString(IMU_CONFIG)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SensAItion: Failed to parse IMU config");
    } else {
        // Store IMU config
        imu_candidate.config = config_;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: IMU config parsed - %d bytes expected", imu_candidate.config.num_bytes);
    }
    
    if (!parseConfigString(AHRS_CONFIG)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SensAItion: Failed to parse AHRS config");  
    } else {
        // Store AHRS config
        ahrs_candidate.config = config_;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: AHRS config parsed - %d bytes expected", ahrs_candidate.config.num_bytes);
    }
    
    // Start with no active config (will be determined by packet validation)  
    active_config = nullptr;
    
    // Use IMU config as default during detection phase
    config_ = imu_candidate.config;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Using IMU config as default during detection");
}

// ArduPilot integration: Convert SensAItion IMU data to ArduPilot format
// Handles accelerometer, gyroscope, barometer, and magnetometer data
void AP_ExternalAHRS_SensAItion::update_ardupilot_imu_data() {
    double accel_x = 0, accel_y = 0, accel_z = 0;
    double gyro_x = 0, gyro_y = 0, gyro_z = 0;
    double temp_val = 0;
    
    Vector3f accel_data, gyro_data;
    bool have_imu_data = false;
    static uint32_t debug_imu = 0;
    bool debug_this = (++debug_imu % 2000 == 0);
    
    if (getSensorValue(0x00, accel_x) && getSensorValue(0x01, accel_y) && getSensorValue(0x02, accel_z)) {
        // Convert from g to m/s²
        accel_data = Vector3f(accel_x * 9.80665, accel_y * 9.80665, accel_z * 9.80665);
        have_imu_data = true;
        
        {
            WITH_SEMAPHORE(state.sem);
            state.accel = accel_data;
        }
        
        if (debug_this) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Accel %.2f,%.2f,%.2f g", accel_x, accel_y, accel_z);
        }
    } else if (debug_this) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: No accel data from sensors 0x00-0x02");
    }
    
    if (getSensorValue(0x03, gyro_x) && getSensorValue(0x04, gyro_y) && getSensorValue(0x05, gyro_z)) {
        // Convert from deg/s to rad/s  
        gyro_data = Vector3f(radians(gyro_x), radians(gyro_y), radians(gyro_z));
        have_imu_data = true;
        
        {
            WITH_SEMAPHORE(state.sem);
            state.gyro = gyro_data;
        }
        
        if (debug_this) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Gyro %.2f,%.2f,%.2f deg/s", gyro_x, gyro_y, gyro_z);
        }
    } else if (debug_this) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: No gyro data from sensors 0x03-0x05");
    }
    
    // Handle external barometer data
    double pressure = 0;
    if (getSensorValue(0x0D, pressure) && getSensorValue(0x09, temp_val)) {
#if AP_BARO_EXTERNALAHRS_ENABLED
        AP_ExternalAHRS::baro_data_message_t baro;
        baro.instance = 0;
        baro.pressure_pa = pressure * 100.0; // Convert mbar to Pa
        baro.temperature = temp_val;
        
        AP::baro().handle_external(baro);
        
        if (debug_this) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Sent baro %.1f mbar, %.1f°C", pressure, temp_val);
        }
#endif
    } else if (debug_this) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: No baro data from sensors 0x0D/0x09");
    }
    
    // Handle external magnetometer data
    double mag_x = 0, mag_y = 0, mag_z = 0;
    if (getSensorValue(0x0A, mag_x) && getSensorValue(0x0B, mag_y) && getSensorValue(0x0C, mag_z)) {
#if AP_COMPASS_EXTERNALAHRS_ENABLED
        AP_ExternalAHRS::mag_data_message_t mag;
        mag.field = Vector3f(mag_x, mag_y, mag_z); // Already in mGauss units
        
        AP::compass().handle_external(mag);
        
        if (debug_this) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Sent mag %.1f,%.1f,%.1f mGauss", mag_x, mag_y, mag_z);
        }
#endif
    } else if (debug_this) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: No mag data from sensors 0x0A-0x0C");
    }
    
    // Send IMU data to ArduPilot's INS system, is this needed or is state.acc/state.gyro enough?
    if (have_imu_data) {
        AP_ExternalAHRS::ins_data_message_t ins;
        ins.accel = accel_data;
        ins.gyro = gyro_data;
        ins.temperature = temp_val;
        
        AP::ins().handle_external(ins);
        
        if (debug_this) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Sent external IMU data to ArduPilot");
        }
    } else if (debug_this) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: No IMU data to send to ArduPilot");
    }
}

// ArduPilot integration: Convert SensAItion AHRS quaternion data to ArduPilot format
// Only available when SensAItion is in AHRS mode (includes quaternion sensors)
void AP_ExternalAHRS_SensAItion::update_ardupilot_attitude_data() {
    double quat_w = 0, quat_x = 0, quat_y = 0, quat_z = 0;
    static uint32_t debug_att = 0;
    bool debug_this = (++debug_att % 2000 == 0);
    
    // Get quaternion from sensors 0x4C-0x4F (AHRS mode only)
    if (getSensorValue(0x4C, quat_w) && getSensorValue(0x4D, quat_x) && 
        getSensorValue(0x4E, quat_y) && getSensorValue(0x4F, quat_z)) {
        
        // Debug before creating Quaternion
        static uint32_t quat_create_debug = 0;
        if (++quat_create_debug % 100 == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Pre-Quat values: w=%.6f, x=%.6f, y=%.6f, z=%.6f", 
                          quat_w, quat_x, quat_y, quat_z);
        }
        
        state.quat = Quaternion(quat_w, quat_x, quat_y, quat_z);
        state.have_quaternion = true;
        
        if (debug_this) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SensAItion: Quat %.3f,%.3f,%.3f,%.3f", quat_w, quat_x, quat_y, quat_z);
        }
    } else if (debug_this) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: No quaternion data from sensors 0x4C-0x4F");
    }
#if AP_COMPASS_EXTERNALAHRS_ENABLED
    double mag_x = 0, mag_y = 0, mag_z = 0;
    if (getSensorValue(0x0A, mag_x) && getSensorValue(0x0B, mag_y) && getSensorValue(0x0C, mag_z)) {
        AP_ExternalAHRS::mag_data_message_t mag;
        // Convert mGauss to Gauss then back to mGauss for handle_external  
        mag.field = Vector3f(mag_x, mag_y, mag_z); // mGauss units
        AP::compass().handle_external(mag);
    }
#endif
}

// Config validation and switching logic, removed later if auto detection is not needed, i.e we define config in Mission Planner
bool AP_ExternalAHRS_SensAItion::validateConfigCandidate(ConfigCandidate& candidate, const uint8_t* packet, size_t packet_size) {
    // Validate inputs
    if (packet == nullptr || packet_size == 0) {
        return false;
    }
    
    // Validate candidate config
    if (candidate.config.num_bytes <= 0 || candidate.config.num_bytes > MAX_PACKET_SIZE) {
        return false;
    }
    
    // Validate packet size matches config expectation
    size_t expected_size = candidate.config.num_bytes + 1; // +1 for header
    if (packet_size != expected_size) {
        return false;
    }
    
    // Validate header
    if (packet[0] != HEADER_BYTE) {
        return false;
    }
    
    // Validate checksum if config uses it
    if (candidate.config.using_checksum) {
        uint8_t calculated = calculateXorChecksum(packet, 1, packet_size - 2);
        uint8_t received = packet[packet_size - 1];
        if (calculated != received) {
            return false;
        }
    }
    
    // Additional validation: check if packet layout makes sense
    // Ensure data layout count matches expected bytes
    if (candidate.config.data_layout_count + (candidate.config.using_checksum ? 1 : 0) != candidate.config.num_bytes) {
        return false;
    }
    
    return true;
}

void AP_ExternalAHRS_SensAItion::updateCandidateSuccess(ConfigCandidate& candidate) {
    candidate.consecutive_successes++;
    candidate.consecutive_failures = 0;
    candidate.last_success_ms = AP_HAL::millis();
}

void AP_ExternalAHRS_SensAItion::updateCandidateFailure(ConfigCandidate& candidate) {
    candidate.consecutive_failures++;
    candidate.consecutive_successes = 0;
}

bool AP_ExternalAHRS_SensAItion::shouldSwitchConfig() {
    // Don't switch if we have a locked config that's still working
    if (active_config && active_config->consecutive_failures < UNLOCK_FAILURES) {
        return false;
    }
    
    // Switch if any candidate has reached lock threshold
    if (imu_candidate.consecutive_successes >= LOCK_THRESHOLD && 
        (!active_config || active_config != &imu_candidate)) {
        return true;
    }
    
    if (ahrs_candidate.consecutive_successes >= LOCK_THRESHOLD && 
        (!active_config || active_config != &ahrs_candidate)) {
        return true;
    }
    
    // Force switch if current config is failing badly
    if (active_config && active_config->consecutive_failures >= UNLOCK_FAILURES) {
        return true;
    }
    
    return false;
}

#endif  // AP_EXTERNAL_AHRS_SENSAITION_ENABLED