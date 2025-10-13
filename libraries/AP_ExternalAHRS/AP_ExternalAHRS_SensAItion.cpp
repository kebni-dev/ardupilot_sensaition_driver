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
    parser(get_config_mode() == ConfigMode::CONFIG_MODE_IMU ?
           AP_ExternalAHRS_SensAItion_Parser::ConfigMode::CONFIG_MODE_IMU :
           AP_ExternalAHRS_SensAItion_Parser::ConfigMode::CONFIG_MODE_AHRS)
{
    // SensAItion provides IMU data only (no GPS/position/attitude data)
    {
        WITH_SEMAPHORE(state.sem);
        state.have_location = false;
        state.have_velocity = false;
        // Set dummy origin to pass prearm checks - actual origin comes from GPS
        state.origin = Location{0, 0, 1, Location::AltFrame::ABSOLUTE};
        state.have_origin = true;
        state.have_quaternion = false;
    }

    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart || baudrate == 0 || port_num == -1) {
        AP_HAL::panic("SensAItion: No UART configured for AHRS protocol");
    }

    // Create thread for non-blocking UART processing
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SensAItion::update_thread, void),
            "AHRS_SensAItion", 2048, AP_HAL::Scheduler::PRIORITY_UART, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SensAItion thread creation failed");
        AP_HAL::panic("SensAItion Failed to start ExternalAHRS update thread");
    }
}

// Extract sensor data and update state
bool AP_ExternalAHRS_SensAItion::extract_sensor_data(const uint8_t* packet)
{
    WITH_SEMAPHORE(state.sem);

    // TEMPORARY DEBUG: Print raw packet bytes (first 20 bytes only, every 100th packet)
    static uint32_t temp_debug_counter = 0;
    if (++temp_debug_counter % 100 == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TEMP_DEBUG_RAW_PACKET: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X ...",
                      packet[0], packet[1], packet[2], packet[3], packet[4], packet[5], packet[6], packet[7], packet[8], packet[9],
                      packet[10], packet[11], packet[12], packet[13], packet[14], packet[15], packet[16], packet[17], packet[18], packet[19]);

        // Show byte order interpretation
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TEMP_DEBUG_BYTE_ORDER: accel_x bytes: [0]=%02X [1]=%02X [2]=%02X [3]=%02X",
                      packet[0], packet[1], packet[2], packet[3]);
    }

    // Extract accelerometer (µg to g, 1e-6 scale)
    // Header already removed by parser
    int32_t accel_x_ug = (packet[0]<<24)|(packet[1]<<16)|(packet[2]<<8)|packet[3];
    int32_t accel_y_ug = (packet[4]<<24)|(packet[5]<<16)|(packet[6]<<8)|packet[7];
    int32_t accel_z_ug = (packet[8]<<24)|(packet[9]<<16)|(packet[10]<<8)|packet[11];

    // TEMPORARY DEBUG: Print raw integer values before conversion
    if (temp_debug_counter % 100 == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TEMP_DEBUG_ACCEL_RAW: x=%d y=%d z=%d ug", (int)accel_x_ug, (int)accel_y_ug, (int)accel_z_ug);
    }

    Vector3f accel_g(accel_x_ug * 1e-6f, accel_y_ug * 1e-6f, accel_z_ug * 1e-6f);
    state.accel = accel_g * GRAVITY_MSS; // Convert g to m/s^2

    // Extract and scale gyroscope (µdeg/s to deg/s, 1e-6 scale, then to rad/s)
    int32_t gyro_x_raw = (packet[12]<<24)|(packet[13]<<16)|(packet[14]<<8)|packet[15];
    int32_t gyro_y_raw = (packet[16]<<24)|(packet[17]<<16)|(packet[18]<<8)|packet[19];
    int32_t gyro_z_raw = (packet[20]<<24)|(packet[21]<<16)|(packet[22]<<8)|packet[23];

    // TEMPORARY DEBUG: Print raw gyro values
    if (temp_debug_counter % 100 == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TEMP_DEBUG_GYRO_RAW: x=%d y=%d z=%d udeg/s", (int)gyro_x_raw, (int)gyro_y_raw, (int)gyro_z_raw);
    }

    Vector3f gyro_degs(gyro_x_raw * 1e-6f, gyro_y_raw * 1e-6f, gyro_z_raw * 1e-6f);
    Vector3f gyro_rads(radians(gyro_degs.x), radians(gyro_degs.y), radians(gyro_degs.z));
    state.gyro = gyro_rads;

    // Extract and scale temperature (2 bytes, special formula)
    int16_t temp_raw = (packet[24]<<8)|packet[25];
    float temp_degc = static_cast<float>(temp_raw) * 0.008f + 20.0f;

    // Send IMU data
    AP_ExternalAHRS::ins_data_message_t ins;
    ins.accel = state.accel;
    ins.gyro = state.gyro;
    ins.temperature = temp_degc;

    // TEMPORARY DEBUG: Print final values
    if (temp_debug_counter % 100 == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TEMP_DEBUG_FINAL: accel=[%.3f,%.3f,%.3f]m/s2 gyro=[%.3f,%.3f,%.3f]rad/s temp=%.1fC",
                      (double)state.accel.x, (double)state.accel.y, (double)state.accel.z,
                      (double)state.gyro.x, (double)state.gyro.y, (double)state.gyro.z, (double)temp_degc);
    }

    AP::ins().handle_external(ins);

    // Extract magnetometer (2 bytes each, already in mGauss)
    int16_t mag_x_mgauss = (packet[26]<<8)|packet[27];
    int16_t mag_y_mgauss = (packet[28]<<8)|packet[29];
    int16_t mag_z_mgauss = (packet[30]<<8)|packet[31];

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS::mag_data_message_t mag;
    mag.field = Vector3f(mag_x_mgauss, mag_y_mgauss, mag_z_mgauss);
    AP::compass().handle_external(mag);
#endif

    // Extract and scale barometer (4 bytes, sensor sends in units of 0.1 Pa)
    int32_t baro_raw = (packet[32]<<24)|(packet[33]<<16)|(packet[34]<<8)|packet[35];
    float pressure_p = baro_raw * 0.1f; // sensor units 0.1 Pa → Pa

    // TEMPORARY DEBUG: Print barometer values
    if (temp_debug_counter % 100 == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TEMP_DEBUG_BARO: raw=%d pressure_Pa=%.1f bytes=[%02X %02X %02X %02X]",
                      (int)baro_raw, (double)pressure_p, packet[32], packet[33], packet[34], packet[35]);
    }

#if AP_BARO_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS::baro_data_message_t baro;
    baro.instance = 0;
    baro.pressure_pa = pressure_p;
    baro.temperature = temp_degc;
    AP::baro().handle_external(baro);
#endif

    // AHRS mode: extract quaternion (sensors 0x0D-0x10, bytes 36-51)
    if (get_config_mode() == ConfigMode::CONFIG_MODE_AHRS) {
        int32_t quat_w_raw = (packet[36]<<24)|(packet[37]<<16)|(packet[38]<<8)|packet[39];
        int32_t quat_x_raw = (packet[40]<<24)|(packet[41]<<16)|(packet[42]<<8)|packet[43];
        int32_t quat_y_raw = (packet[44]<<24)|(packet[45]<<16)|(packet[46]<<8)|packet[47];
        int32_t quat_z_raw = (packet[48]<<24)|(packet[49]<<16)|(packet[50]<<8)|packet[51];

        state.quat = Quaternion(quat_w_raw * 1e-6f, quat_x_raw * 1e-6f,
                                quat_y_raw * 1e-6f, quat_z_raw * 1e-6f);
        state.have_quaternion = true;
    } else {
        state.have_quaternion = false;
    }

    return true;
}

// Interface methods
int8_t AP_ExternalAHRS_SensAItion::get_port() const
{
    if (!uart) {
        return -1;
    }
    return port_num;
}

const char* AP_ExternalAHRS_SensAItion::get_name() const
{
    return "Kebni SensAItion";
}

bool AP_ExternalAHRS_SensAItion::healthy() const
{
    uint32_t now_ms = AP_HAL::millis();
    return (now_ms - last_valid_packet_ms) < 1000 && valid_packets > 0;
}

bool AP_ExternalAHRS_SensAItion::initialised() const
{
    return setup_complete;
}

bool AP_ExternalAHRS_SensAItion::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SensAItion unhealthy");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_SensAItion::get_filter_status(nav_filter_status &status) const
{
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
void AP_ExternalAHRS_SensAItion::update_thread()
{
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay(1);
        }
    }
}

bool AP_ExternalAHRS_SensAItion::check_uart()
{
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

    // Read up to MAX_PACKET_SIZE bytes at a time
    uint8_t buffer[AP_ExternalAHRS_SensAItion_Parser::MAX_PACKET_SIZE];
    n = MIN(n, sizeof(buffer));
    ssize_t nread = uart->read(buffer, n);

    if (nread > 0) {
        // Use parser to get validated packets
        const uint8_t* packet;
        size_t packet_size;
        if (parser.parse_bytes(buffer, nread, packet, packet_size)) {
            // Extract sensor data from validated packet
            if (extract_sensor_data(packet)) {
                valid_packets++;
                last_valid_packet_ms = AP_HAL::millis();

                // TEMPORARY DEBUG: Print packet rate and stats (every 1000 packets = ~1 second)
                static uint32_t temp_last_print_ms = 0;
                if (valid_packets % 1000 == 0) {
                    uint32_t now = AP_HAL::millis();
                    uint32_t dt_ms = now - temp_last_print_ms;
                    uint32_t errors = parser.get_parse_errors();
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TEMP_DEBUG_STATS: %u pkts, %u errs, dt=%ums (%.1fHz)",
                                  (unsigned)valid_packets, (unsigned)errors, (unsigned)dt_ms,
                                  (double)(dt_ms > 0 ? 1000000.0f / dt_ms : 0.0f));
                    temp_last_print_ms = now;
                }

                return true;
            }
        }
    }

    return false;
}

#endif  // AP_EXTERNAL_AHRS_SENSAITION_ENABLED