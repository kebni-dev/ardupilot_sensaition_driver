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
#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_SensAItion::AP_ExternalAHRS_SensAItion(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state),
    parser(AP_ExternalAHRS_SensAItion_Parser::ConfigMode::IMU)
{
    // Determine Mode from EAHRS_OPTIONS
    // Bit 1: Interleaved INS Mode (1) vs IMU Only Mode (0)
    _ins_mode_enabled = option_is_set(static_cast<AP_ExternalAHRS::OPTIONS>(1U << 1));
    
    auto mode = _ins_mode_enabled ? 
                AP_ExternalAHRS_SensAItion_Parser::ConfigMode::INTERLEAVED_INS : 
                AP_ExternalAHRS_SensAItion_Parser::ConfigMode::IMU;

    // Re-initialize logic if mode differs from default IMU
    if (_ins_mode_enabled) {
         parser = AP_ExternalAHRS_SensAItion_Parser(mode);
    }

    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart || baudrate == 0 || port_num == -1) {
        return;
    }

    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SensAItion::update_thread, void),
            "AHRS_SensAItion", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
    }
}

int8_t AP_ExternalAHRS_SensAItion::get_port() const {
    return uart ? port_num : -1;
}

const char* AP_ExternalAHRS_SensAItion::get_name() const {
    return "Kebni SensAItion";
}

uint8_t AP_ExternalAHRS_SensAItion::num_gps_sensors() const {
    // Only expose GPS if we are configured for INS mode and receiving data
    return _ins_mode_enabled ? 1 : 0;
}

bool AP_ExternalAHRS_SensAItion::healthy() const {
    uint32_t now_ms = AP_HAL::millis();

    // 1. IMU Freshness Check (Required for both modes)
    if ((now_ms - _last_imu_pkt_ms) > 40) {
        return false;
    }

    // 2. INS Specific Checks
    if (_ins_mode_enabled) {
        if ((now_ms - _last_ins_pkt_ms) > 200) {
            return false;
        }

        // Hardware Health: Sensor Valid (Byte 49)
        // Bit 0: IMU Available. Must be 1.
        if (!(_last_sensor_valid & 0x01)) {
            return false;
        }

        // GNSS Status (Option A: Strict from Spec 5.1)
        // "Packet 2 GNSS1 Fix > 3 (3D Fix)" - Assuming 3 is 3D Fix in Ublox convention
        if (_last_gnss1_fix < 3) {
            return false;
        }
    }

    return true;
}

bool AP_ExternalAHRS_SensAItion::initialised() const {
    return setup_complete;
}

bool AP_ExternalAHRS_SensAItion::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const {
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SensAItion Unhealthy");
        return false;
    }

    if (_ins_mode_enabled) {
        // Spec 5.2: Alignment Status
        if (_last_alignment_status != 1) {
            hal.util->snprintf(failure_msg, failure_msg_len, "SensAItion Aligning");
            return false;
        }

        // Spec 5.2: Quality Gates
        // if (_last_pos_acc > 5.0f) {
        //     hal.util->snprintf(failure_msg, failure_msg_len, "SensAItion High Pos Var");
        //     return false;
        // }
        // if (_last_vel_acc > 1.5f) {
        //     hal.util->snprintf(failure_msg, failure_msg_len, "SensAItion High Vel Var");
        //     return false;
        // }
    }

    return true;
}

void AP_ExternalAHRS_SensAItion::get_filter_status(nav_filter_status &status) const {
    WITH_SEMAPHORE(state.sem);
    memset(&status, 0, sizeof(status));
    
    // Basic initialization based on successful parsing
    status.flags.initalized = initialised();
    if (healthy()) {
        if (_ins_mode_enabled && _last_alignment_status == 1) {
            status.flags.attitude = true;
            status.flags.horiz_pos_abs = true;
            status.flags.vert_pos = true;
            status.flags.horiz_vel = true;
            status.flags.vert_vel = true;
            status.flags.using_gps = true;
            status.flags.horiz_pos_rel = true;
            
            
            // "Pred" flags indicate the filter can predict position
            status.flags.pred_horiz_pos_abs = true;
            status.flags.pred_horiz_pos_rel = true;
        }
    }
}

void AP_ExternalAHRS_SensAItion::update_thread() {
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay_microseconds(100);
        }
    }
}

bool AP_ExternalAHRS_SensAItion::check_uart() {
    if (!uart) return false;

    if (!setup_complete) {
        uart->begin(baudrate);
        setup_complete = true;
    }
    uint32_t n = uart->available();
    if (n == 0) return false;

    // Limit read to buffer size
    n = MIN(n, sizeof(buffer));
    ssize_t nread = uart->read(buffer, n);
    
    bool parsed_any = false;

    if (nread > 0) {
        parser.parse_bytes(buffer, nread, sensor_measurement);
        
        // If UNINITIALIZED, parser needs more data or found nothing
        if (sensor_measurement.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::UNINITIALIZED) {
            return false;
        }

        parsed_any = true;
        uint32_t now = AP_HAL::millis();

        // --- PACKET ROUTING & STATE UPDATE ---

        // 1. IMU DATA
        if (sensor_measurement.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::IMU) {

            // Update shared state for frontend (AHRS fallback)
            {
                WITH_SEMAPHORE(state.sem);
                state.accel = sensor_measurement.acceleration_mss;
                state.gyro = sensor_measurement.angular_velocity_rads;
            }
            _last_imu_pkt_ms = now;

            AP_ExternalAHRS::ins_data_message_t ins;
            ins.accel = sensor_measurement.acceleration_mss;
            ins.gyro = sensor_measurement.angular_velocity_rads;
            ins.temperature = sensor_measurement.temperature_degc;
            AP::ins().handle_external(ins);

#if AP_COMPASS_EXTERNALAHRS_ENABLED
            AP_ExternalAHRS::mag_data_message_t mag;
            mag.field = sensor_measurement.magnetic_field_mgauss;
            AP::compass().handle_external(mag);
#endif
#if AP_BARO_EXTERNALAHRS_ENABLED
            AP_ExternalAHRS::baro_data_message_t baro;
            baro.instance = 0;
            baro.pressure_pa = sensor_measurement.air_pressure_p;
            baro.temperature = sensor_measurement.temperature_degc;
            AP::baro().handle_external(baro);
#endif
        
        }

        // 2. AHRS (QUATERNION)
        else if (sensor_measurement.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::AHRS) {
            _last_quat_pkt_ms = now;
            WITH_SEMAPHORE(state.sem);
            state.quat = sensor_measurement.orientation;
            state.have_quaternion = true;
        }

        // 3. INS / GPS / STATUS
        else if (sensor_measurement.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::INS) {
            _last_ins_pkt_ms = now;

            // Update Persistent State Variables (Critical for healthy() checks)
            _last_alignment_status = sensor_measurement.alignment_status;
            _last_sensor_valid = sensor_measurement.sensor_valid;
            _last_gnss1_fix = sensor_measurement.gnss1_fix;
            _last_gnss2_fix = sensor_measurement.gnss2_fix;
            _last_error_flags = sensor_measurement.error_flags;
            _last_pos_acc = sensor_measurement.pos_accuracy_horiz;
            _last_vel_acc = sensor_measurement.vel_accuracy;

            // Log Diagnostics (Spec 6.1)
            log_ins_status(sensor_measurement);

            // Populate GPS data structure
            AP_ExternalAHRS::gps_data_message_t gps;
            gps.gps_week = 2396; // Not extracted
            gps.ms_tow = sensor_measurement.time_itow;
            gps.fix_type = (AP_GPS_FixType)sensor_measurement.gnss1_fix;
            gps.satellites_in_view = sensor_measurement.num_sats_gnss1;

            gps.horizontal_pos_accuracy = sensor_measurement.pos_accuracy_horiz;
            gps.vertical_pos_accuracy = sensor_measurement.pos_accuracy_vert;
            gps.horizontal_vel_accuracy = sensor_measurement.vel_accuracy;

            gps.latitude = sensor_measurement.location.lat;
            gps.longitude = sensor_measurement.location.lng;
            gps.msl_altitude = sensor_measurement.location.alt; // cm
            // gps.hdop = 1;
            // gps.vdop = 1; // TODO // I think ardupilot ignores dop if accuracy fields are set

            gps.ned_vel_north = sensor_measurement.velocity_ned.x;
            gps.ned_vel_east = sensor_measurement.velocity_ned.y;
            gps.ned_vel_down = sensor_measurement.velocity_ned.z;

            // Inject into ArduPilot GPS backend
            uint8_t instance;
            if (AP::gps().get_first_external_instance(instance)) {
                AP::gps().handle_external(gps, instance);
            }

            // Update shared state for EKF consumption
            WITH_SEMAPHORE(state.sem);
            state.location = Location(
                sensor_measurement.location.lat,
                sensor_measurement.location.lng,
                sensor_measurement.location.alt,
                Location::AltFrame::ABSOLUTE
            );
            state.velocity = sensor_measurement.velocity_ned;
            state.have_location = true;
            state.have_velocity = true;
            state.last_location_update_us = AP_HAL::micros();

            // Only claim origin if alignment is complete and sensor is healthy and navigation solution is happy
            if (!state.have_origin && sensor_measurement.alignment_status)
            {
                WITH_SEMAPHORE(state.sem);
                state.origin = Location{int32_t(sensor_measurement.location.lat),
                        int32_t(sensor_measurement.location.lng),
                        int32_t(sensor_measurement.location.alt),
                        Location::AltFrame::ABSOLUTE};
                state.have_origin = true;
            }
        }
    }
    
    return parsed_any;
}

bool AP_ExternalAHRS_SensAItion::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    WITH_SEMAPHORE(state.sem);
    
    // Safety: Only report variances if we are in INS mode and Aligned
    if (_ins_mode_enabled && _last_alignment_status == 1) {
        
        // [CPO FIX] Math: Variance = Accuracy^2 (Standard Deviation Squared)
        // _last_pos_acc is in meters (from Packet 2)
        // _last_vel_acc is in m/s (from Packet 2)
        posVar = sq(_last_pos_acc);
        velVar = sq(_last_vel_acc);
        
        // Vertical accuracy: Packet 2 only gives "Quality Pos" (Val 97).
        // We assume Vertical error is roughly similar to Horizontal for this sensor class.
        hgtVar = posVar; 
        
        // Mag variance: We trust the external mag calibration
        magVar.zero(); 
        
        // Airspeed variance: Unknown/Unused
        tasVar = 0;
        
        return true;
    }
    
    // Fallback: If not aligned or in IMU mode, we cannot provide INS variances.
    return false;
}

// [CPO ARCHITECT] Unified DataFlash Logging
// Logs Attitude (from Packet 1) + INS Status (from Packet 2) in a single synchronized message.
void AP_ExternalAHRS_SensAItion::log_ins_status(const AP_ExternalAHRS_SensAItion_Parser::Measurement &meas)
{
#if HAL_LOGGING_ENABLED
    // Only log if we actually have INS data (Packet 2)
    // This ensures we don't log empty/stale data if called incorrectly
    if (meas.type != AP_ExternalAHRS_SensAItion_Parser::MeasurementType::INS) {
        return;
    }

    // 1. Get Attitude from the shared state (Updated by Packet 1 just before this)
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    {
        WITH_SEMAPHORE(state.sem);
        if (state.have_quaternion) {
            state.quat.to_euler(roll_rad, pitch_rad, yaw_rad);
        }
    }

    // 2. Write to DataFlash
    // @LoggerMessage: KEB1
    // @Description: Kebni SensAItion Fusion Status
    // @Field: TimeUS: Time since system startup
    // @Field: Roll: Roll angle (deg)
    // @Field: Pitch: Pitch angle (deg)
    // @Field: Yaw: Yaw angle (deg)
    // @Field: Align: Alignment Status (1=Aligned)
    // @Field: Fix1: GNSS1 Fix Type
    // @Field: Fix2: GNSS2 Fix Type
    // @Field: Err: Error Flags Bitmask
    // @Field: Val: Sensor Valid Bitmask
    AP::logger().Write("KEB1", "TimeUS,Roll,Pitch,Yaw,Align,Fix1,Fix2,Err,Val", "QfffBHHIB",
                       AP_HAL::micros64(),
                       (double)degrees(roll_rad),
                       (double)degrees(pitch_rad),
                       (double)degrees(yaw_rad),
                       (uint8_t)meas.alignment_status,
                       (uint16_t)meas.gnss1_fix,
                       (uint16_t)meas.gnss2_fix,
                       (uint32_t)meas.error_flags,
                       (uint8_t)meas.sensor_valid);
#endif
}

#endif // AP_EXTERNAL_AHRS_SENSAITION_ENABLED