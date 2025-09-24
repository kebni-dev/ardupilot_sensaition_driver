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
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_SensAItion::AP_ExternalAHRS_SensAItion(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state),
    parse_state_(LOOKING_FOR_HEADER),
    packet_buffer_len_(0),
    uart(nullptr),
    baudrate(460800),
    port_num(-1),
    setup_complete(false),
    last_valid_packet_ms(0),
    valid_packets(0)
{

    // Tell ardupilot that we do not provide location or velocity
      state.have_location = false;
      state.have_velocity = false;
      state.have_origin = true; // OK?

    // Allocate packet buffer
    packet_buffer_ = NEW_NOTHROW uint8_t[MAX_PACKET_SIZE];
    if (!packet_buffer_) {
        return;
    }
    
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    // Create thread for non-blocking UART processing
    if (!hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SensAItion::update_thread, void),
        "AHRS_SensAItion", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SensAItion thread creation failed");
        AP_HAL::panic("SensAItion Failed to start ExternalAHRS update thread");
    }
}



//SensAItion parser 
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
            
            // Simple fixed packet size collection based on current mode
            size_t expected_size = (getConfigMode() == CONFIG_MODE_IMU) ? 38 : 54;

            if (packet_buffer_len_ == expected_size) {
                // Parse packet with mode-specific parser
                if (parsePacket(packet_buffer_, packet_buffer_len_)) {
                    processed_any = true;
                    valid_packets++;
                    last_valid_packet_ms = AP_HAL::millis();
                }
                parse_state_ = LOOKING_FOR_HEADER;
            } else if (packet_buffer_len_ > expected_size) {
                // Packet too large, reset
                parse_state_ = LOOKING_FOR_HEADER;
            }
            // If packet_buffer_len_ < expected_size, keep collecting
            
            if (packet_buffer_len_ > MAX_PACKET_SIZE) {
                // Prevent buffer overflow
                parse_state_ = LOOKING_FOR_HEADER;
            }
            break;
        }
    }
    
    return processed_any;
}

// Packet parser 
bool AP_ExternalAHRS_SensAItion::parsePacket(const uint8_t* packet, size_t packet_size) {
    // Get expected size based on mode
    size_t expected_size = (getConfigMode() == CONFIG_MODE_IMU) ? 38 : 54;
    const char* mode_name = (getConfigMode() == CONFIG_MODE_IMU) ? "IMU" : "AHRS";

    // Validate packet structure
    if (packet_size != expected_size || packet[0] != HEADER_BYTE) {
        static uint32_t parse_errors = 0;
        if (++parse_errors % 1000 == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SensAItion: Parse failed, expect %s mode (%u bytes, header 0xFA), got %u bytes, header 0x%02X",
                         mode_name, (unsigned)expected_size, (unsigned)packet_size, packet[0]);
        }
        return false;
    }

    // Validate checksum
    uint8_t calculated = calculateXorChecksum(packet, 1, packet_size - 2);
    uint8_t received = packet[packet_size - 1];
    if (calculated != received) {
        return false;
    }

    return extractSensorData(packet);
}

// Sensor data extraction, scaling and update ardupilot state
bool AP_ExternalAHRS_SensAItion::extractSensorData(const uint8_t* packet) {
    WITH_SEMAPHORE(state.sem);

    // Extract and scale accelerometer (µg to g, 1e-6 scale)
    int32_t accel_x_raw = (packet[1]<<24)|(packet[2]<<16)|(packet[3]<<8)|packet[4];
    int32_t accel_y_raw = (packet[5]<<24)|(packet[6]<<16)|(packet[7]<<8)|packet[8];
    int32_t accel_z_raw = (packet[9]<<24)|(packet[10]<<16)|(packet[11]<<8)|packet[12];

    Vector3f accel_data(accel_x_raw * 1e-6f, accel_y_raw * 1e-6f, accel_z_raw * 1e-6f);
    accel_data *= GRAVITY_MSS; // Convert g to m/s^2
    state.accel = accel_data;

    // Extract and scale gyroscope (µdeg/s to deg/s, 1e-6 scale, then to rad/s)
    int32_t gyro_x_raw = (packet[13]<<24)|(packet[14]<<16)|(packet[15]<<8)|packet[16];
    int32_t gyro_y_raw = (packet[17]<<24)|(packet[18]<<16)|(packet[19]<<8)|packet[20];
    int32_t gyro_z_raw = (packet[21]<<24)|(packet[22]<<16)|(packet[23]<<8)|packet[24];

    Vector3f gyro_deg(gyro_x_raw * 1e-6f, gyro_y_raw * 1e-6f, gyro_z_raw * 1e-6f);
    Vector3f gyro_data(radians(gyro_deg.x), radians(gyro_deg.y), radians(gyro_deg.z));
    state.gyro = gyro_data;

    // Extract and scale temperature (2 bytes, special formula)
    int16_t temp_raw = (packet[25]<<8)|packet[26];
    float temp_celsius = (static_cast<float>(temp_raw) / 10000.0f) * 80.0f + 20.0f;

    // Send IMU data to ArduPilot
    AP_ExternalAHRS::ins_data_message_t ins;
    ins.accel = accel_data;
    ins.gyro = gyro_data;
    ins.temperature = temp_celsius;
    AP::ins().handle_external(ins);

    // Extract magnetometer (2 bytes each, already in mGauss)
    int16_t mag_x_raw = (packet[27]<<8)|packet[28];
    int16_t mag_y_raw = (packet[29]<<8)|packet[30];
    int16_t mag_z_raw = (packet[31]<<8)|packet[32];

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS::mag_data_message_t mag;
    mag.field = Vector3f(mag_x_raw, mag_y_raw, mag_z_raw); // Already in mGauss
    AP::compass().handle_external(mag);
#endif

    // Extract and scale barometer (4 bytes, comes in mhPa, convert to Pa)
    int32_t baro_raw = (packet[33]<<24)|(packet[34]<<16)|(packet[35]<<8)|packet[36];
    float pressure_pa = baro_raw * 0.1f; // mhPa to Pa

#if AP_BARO_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS::baro_data_message_t baro;
    baro.instance = 0;
    baro.pressure_pa = pressure_pa;
    baro.temperature = temp_celsius;
    AP::baro().handle_external(baro);
#endif

    // AHRS mode: extract quaternion
    if (getConfigMode() == CONFIG_MODE_AHRS) {
        int32_t quat_w_raw = (packet[37]<<24)|(packet[38]<<16)|(packet[39]<<8)|packet[40];
        int32_t quat_x_raw = (packet[41]<<24)|(packet[42]<<16)|(packet[43]<<8)|packet[44];
        int32_t quat_y_raw = (packet[45]<<24)|(packet[46]<<16)|(packet[47]<<8)|packet[48];
        int32_t quat_z_raw = (packet[49]<<24)|(packet[50]<<16)|(packet[51]<<8)|packet[52];

        float quat_w = quat_w_raw * 1e-6f;
        float quat_x = quat_x_raw * 1e-6f;
        float quat_y = quat_y_raw * 1e-6f;
        float quat_z = quat_z_raw * 1e-6f;

        state.quat = Quaternion(quat_w, quat_x, quat_y, quat_z);
        state.have_quaternion = true;
    } else {
        state.have_quaternion = false;
    }


    return true;
}



// Utility methods
uint8_t AP_ExternalAHRS_SensAItion::calculateXorChecksum(const uint8_t* data, size_t start, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = start; i < start + length && i < MAX_PACKET_SIZE; i++) {
        checksum ^= data[i];
    }
    return checksum;
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
    return setup_complete;
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
        status.flags.horiz_vel = false;
        status.flags.vert_vel = false;
        status.flags.horiz_pos_rel = false; // Are these needed if we don't implement INS right now?
        status.flags.horiz_pos_abs = false;
        status.flags.vert_pos = false;
        status.flags.using_gps = false;
        status.flags.gps_glitching = false;
        status.flags.initalized = true;
    }
}

// Thread function for non-blocking UART processing
void AP_ExternalAHRS_SensAItion::update_thread() {
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay(1);
        }
    }
}

// Main update method, now empty since thread handles everything
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
        parseMultipleBytes(buffer, nread);
        return true;
    }
    
    return false;
}

#endif  // AP_EXTERNAL_AHRS_SENSAITION_ENABLED