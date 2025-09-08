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
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // get model/type name
    const char* get_name() const override;

    void update() override;

    uint8_t num_gps_sensors() const override
    {
        return 0;  // SensAItion supports only AHRS and IMU, not INS
    }

private:
    // SensAItion protocol definitions
    enum class DataType {
        FLOAT,      // 32-bit float (scaled integer)
        FLOAT16,    // 16-bit float (scaled integer)
        INT8,       // 8-bit signed integer
        INT16,      // 16-bit signed integer  
        INT32,      // 32-bit signed integer
        UINT8,      // 8-bit unsigned integer
        UINT16,     // 16-bit unsigned integer
        UINT32,     // 32-bit unsigned integer
        FLAGS8,     // 8-bit flags
        FLAGS16,    // 16-bit flags
        FLAGS32,    // 32-bit flags
        UINT16x2,   // Two 16-bit values
        UINT8x4     // Four 8-bit values
    };
    
    // Sensor value definitions
    struct SensorValue {
        const char* name;
        const char* unit;
        DataType type;
        double scale_factor;
        
        // Current data
        union {
            double dfloat;
            int32_t dint;
            uint32_t duint;
            uint16_t duint16x2[2];
            uint8_t duint8x4[4];
        } data;
        
        bool valid = false;
        uint32_t timestamp_ms = 0;
    };
    
    // Configuration parsing for SensAItion data layout strings
    struct ParseConfig {
        int data_rate = 1;
        int num_bytes = -1;
        bool using_checksum = true;
        bool using_big_endian = true;
        
        struct DataEntry {
            int sensor_id;  // Which sensor (0-255)
            int byte_index; // Which byte of that sensor (0-3)
        };
        static constexpr uint8_t MAX_DATA_ENTRIES = 64;
        DataEntry data_layout[MAX_DATA_ENTRIES];
        uint8_t data_layout_count = 0;
    };

    // SensAItion protocol constants
    static constexpr uint8_t HEADER_BYTE = 0xFA;
    static constexpr int MAX_PACKET_SIZE = 256;

    // Configuration and data
    ParseConfig config_;
    static constexpr uint16_t MAX_SENSOR_ID = 255;
    SensorValue sensor_definitions_[MAX_SENSOR_ID + 1];
    SensorValue sensor_values_[MAX_SENSOR_ID + 1];
    
    // Packet parsing state machine
    enum ParseState { LOOKING_FOR_HEADER, COLLECTING_PACKET };
    ParseState parse_state_ = LOOKING_FOR_HEADER;
    uint8_t *packet_buffer_;
    uint16_t packet_buffer_len_ = 0;



    // SensAItion parser methods
    void initializeSensorDefinitions();
    bool parseConfigString(const char* config);
    uint8_t calculateXorChecksum(const uint8_t* data, size_t start, size_t length);
    bool extractSensorData(const uint8_t* packet);
    void updateSensorValue(int sensor_id, int32_t raw_value);
    int32_t combineBytes(const uint8_t* packet, const int* byte_indices, int num_bytes, bool big_endian);
    uint8_t charToHex(char c);
    
    // Parsing methods
    bool parsePacket(const uint8_t* packet, size_t packet_size);
    bool parseMultipleBytes(const uint8_t* data, size_t data_size);
    
    // Data access methods
    bool getSensorValue(int sensor_id, double& value) const;
    bool isSensorValid(int sensor_id) const;
    
    // Utilities
    void clearData();

    // ArduPilot integration
    AP_HAL::UARTDriver *uart;
    uint32_t baudrate;
    int8_t port_num;

    // Configuration strings, removed later if we do not need auto detection, i.e we define config in Mission Planner
    // IMU config: accelerometer, gyro, temperature, magnetometer, barometer
    static constexpr const char* IMU_CONFIG = "o0002s240030020010000130120110100230220210200330320310300430420410400530520510500910900A10A00B10B00C10C00D30D20D10D0x";
    // AHRS config: all IMU sensors plus quaternion
    static constexpr const char* AHRS_CONFIG = "o0004s340030020010000130120110100230220210200330320310300430420410400530520510500910900A10A00B10B00C10C00D30D20D10D04C34C24C14C04D34D24D14D04E34E24E14E04F34F24F14F0x";
    
    // Config detection - automatically detects IMU vs AHRS mode, removed later if not needed --- i.e we define config in Mission Planner
    // Uses packet size validation to determine which configuration is active, removed later if not needed, i.e we define config in Mission Planner
    struct ConfigCandidate {
        ParseConfig config;
        uint8_t consecutive_successes = 0;
        uint8_t consecutive_failures = 0;
        uint32_t last_success_ms = 0;
        bool is_active = false;
        const char* config_name;
        
        ConfigCandidate() = default;
        ConfigCandidate(const char* name) : config_name(name) {}
    };
    
    ConfigCandidate imu_candidate;
    ConfigCandidate ahrs_candidate;
    ConfigCandidate* active_config = nullptr;
    
    static constexpr uint8_t LOCK_THRESHOLD = 3;      // Require 3 consecutive successes, releated to config detection, removed later
    static constexpr uint8_t UNLOCK_FAILURES = 10;    // Unlock after 10 consecutive failures, releated to config detection, removed later
    
    bool configs_initialized = false;

    // ArduPilot integration, config detetection
    bool setup_complete = false;
    uint32_t last_valid_packet_ms = 0;
    
    // Statistics
    uint32_t complete_packets = 0;
    uint32_t valid_packets = 0;
    uint32_t invalid_checksums = 0;
    

    // Core ArduPilot integration methods
    void initialize();
    void update_thread();
    bool check_uart();
    
    // ArduPilot data update methods
    void update_ardupilot_imu_data();
    void update_ardupilot_attitude_data();
    
    // ArduPilot integration - config validation and switching logic, removed later if auto detection is not needed, i.e we define config in Mission Planner
    bool validateConfigCandidate(ConfigCandidate& candidate, const uint8_t* packet, size_t packet_size);
    void updateCandidateSuccess(ConfigCandidate& candidate);
    void updateCandidateFailure(ConfigCandidate& candidate);
    bool shouldSwitchConfig();

};

#endif  // AP_EXTERNAL_AHRS_SENSAITION_ENABLED