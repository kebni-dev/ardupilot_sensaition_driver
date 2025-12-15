/*
   Support for SensAItion serial connected AHRS and IMU
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
#include <stdio.h>

extern const AP_HAL::HAL &hal;

// [CPO] LOGGING FLAGS
static const bool DEBUG_HEX_DUMP = false;
static const bool DEBUG_TIMING   = false;

AP_ExternalAHRS_SensAItion::AP_ExternalAHRS_SensAItion(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state),
    parser(AP_ExternalAHRS_SensAItion_Parser::ConfigMode::IMU)
{
    _ins_mode_enabled = option_is_set(static_cast<AP_ExternalAHRS::OPTIONS>(1U << 1));
    
    auto mode = _ins_mode_enabled ? 
                AP_ExternalAHRS_SensAItion_Parser::ConfigMode::INTERLEAVED_INS : 
                AP_ExternalAHRS_SensAItion_Parser::ConfigMode::IMU;

    if (_ins_mode_enabled) {
         parser = AP_ExternalAHRS_SensAItion_Parser(mode);
    }

    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart || baudrate == 0 || port_num == -1) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "KEBNI: Serial Port Not Found!");
        return;
    }

    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SensAItion::update_thread, void),
            "AHRS_SensAItion", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "KEBNI: Failed to create thread!");
    }
}

int8_t AP_ExternalAHRS_SensAItion::get_port() const {
    return uart ? port_num : -1;
}

const char* AP_ExternalAHRS_SensAItion::get_name() const {
    return "Kebni SensAItion";
}

uint8_t AP_ExternalAHRS_SensAItion::num_gps_sensors() const {
    return _ins_mode_enabled ? 1 : 0;
}

bool AP_ExternalAHRS_SensAItion::healthy() const {
    uint32_t now_ms = AP_HAL::millis();
    bool is_healthy = true;
    const char* reason = "OK"; 

    if ((now_ms - _last_imu_pkt_ms) > 160) { 
        is_healthy = false;
        reason = "IMU Stale";
    }

    if (is_healthy && _ins_mode_enabled) {
        if ((now_ms - _last_ins_pkt_ms) > 400) { 
            is_healthy = false;
            reason = "INS Stale";
        } else if (!(_last_sensor_valid & 0x01)) {
            is_healthy = false;
            reason = "Sensor Invalid (Bit 0)";
        } else if (_last_gnss1_fix < 3) {
            is_healthy = false;
            reason = "GNSS Fix Low";
        }
    }

    static uint32_t last_health_print = 0;
    if (now_ms - last_health_print > 5000) { 
        last_health_print = now_ms;
        if (!is_healthy) {
             GCS_SEND_TEXT(MAV_SEVERITY_INFO, "KEBNI: Health: BAD (%s). IMU_Age:%u INS_Age:%u", 
                reason,
                (unsigned)(now_ms - _last_imu_pkt_ms),
                (unsigned)(now_ms - _last_ins_pkt_ms));
        }
    }

    return is_healthy;
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
        if (_last_alignment_status != 1) {
            hal.util->snprintf(failure_msg, failure_msg_len, "SensAItion Aligning");
            return false;
        }
    }

    return true;
}

void AP_ExternalAHRS_SensAItion::get_filter_status(nav_filter_status &status) const {
    WITH_SEMAPHORE(state.sem);
    memset(&status, 0, sizeof(status));
    
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
            status.flags.pred_horiz_pos_abs = true;
            status.flags.pred_horiz_pos_rel = true;
        }
    }
}

// ---------------------------------------------------------------------------
// THREAD & PROFILER
// ---------------------------------------------------------------------------
void AP_ExternalAHRS_SensAItion::update_thread() {
    while (true) {
        loop_counter++;
        
        // 1. Measure Loop Jitter
        uint32_t now_us = AP_HAL::micros();
        stats_loop_rate.update_interval(now_us);

        // 2. Run Driver & Measure Execution Time
        uint32_t start_exec = AP_HAL::micros();
        bool got_data = check_uart(); 
        uint32_t end_exec = AP_HAL::micros();
        
        double duration = (double)(end_exec - start_exec);

        if (got_data) {
            data_hit_counter++;
            stats_exec_busy.update_val(duration);
        } else {
            stats_exec_idle.update_val(duration);
        }

        // 3. Periodic Reporting (Every 2 seconds)
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - last_stat_report_ms >= 2000) {
            last_stat_report_ms = now_ms;

            float l_mean, l_std, l_max;
            float idle_mean, idle_std, idle_max;
            float busy_mean, busy_std, busy_max;
            
            float imu_mean, imu_std, imu_max, imu_cost_mean, imu_cost_std, imu_cost_max;
            float ahrs_mean, ahrs_std, ahrs_max, ahrs_cost_mean, ahrs_cost_std, ahrs_cost_max;
            float ins_mean, ins_std, ins_max, ins_cost_mean, ins_cost_std, ins_cost_max;
            
            // Gather Snapshots
            stats_loop_rate.get_stats(l_mean, l_std, l_max);
            stats_exec_idle.get_stats(idle_mean, idle_std, idle_max);
            stats_exec_busy.get_stats(busy_mean, busy_std, busy_max);
            
            stats_imu_rate.get_stats(imu_mean, imu_std, imu_max);
            stats_imu_exec.get_stats(imu_cost_mean, imu_cost_std, imu_cost_max);

            stats_ahrs_rate.get_stats(ahrs_mean, ahrs_std, ahrs_max);
            stats_ahrs_exec.get_stats(ahrs_cost_mean, ahrs_cost_std, ahrs_cost_max);

            stats_ins_rate.get_stats(ins_mean, ins_std, ins_max);
            stats_ins_exec.get_stats(ins_cost_mean, ins_cost_std, ins_cost_max);

            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "=== KEBNI PERF (2s) ===");
            
            // Utilization Calculation
            float utilization = 0.0f;
            if (loop_counter > 0) utilization = ((float)data_hit_counter / (float)loop_counter) * 100.0f;
            
            // Loop & Utilization Report
            if (l_mean > 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LOOP: Rate:%.0fHz Jitter:%.0fus Util:%.1f%%", 
                    (double)(1000000.0f / l_mean), (double)l_std, (double)utilization);
            }

            // Idle vs Busy Execution Time
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LOOP TIME: Idle:%.0fus (s:%.0f) | Busy:%.0fus (s:%.0f)", 
                (double)idle_mean, (double)idle_std, (double)busy_mean, (double)busy_std);

            // Packet Detail Report
            if (stats_imu_rate.count > 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "IMU : Int:%.0fus (J:%.0f) | Exec:%.0fus (s:%.0f) n=%u", 
                    (double)imu_mean, (double)imu_std, (double)imu_cost_mean, (double)imu_cost_std, (unsigned)stats_imu_rate.count);
            }
            if (stats_ahrs_rate.count > 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AHRS: Int:%.0fus (J:%.0f) | Exec:%.0fus (s:%.0f) n=%u", 
                    (double)ahrs_mean, (double)ahrs_std, (double)ahrs_cost_mean, (double)ahrs_cost_std, (unsigned)stats_ahrs_rate.count);
            }
            if (stats_ins_rate.count > 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS : Int:%.0fus (J:%.0f) | Exec:%.0fus (s:%.0f) n=%u", 
                    (double)ins_mean, (double)ins_std, (double)ins_cost_mean, (double)ins_cost_std, (unsigned)stats_ins_rate.count);
            }

            // Reset
            stats_loop_rate.reset();
            stats_exec_idle.reset(); stats_exec_busy.reset();
            stats_imu_rate.reset(); stats_imu_exec.reset();
            stats_ahrs_rate.reset(); stats_ahrs_exec.reset();
            stats_ins_rate.reset(); stats_ins_exec.reset();
            loop_counter = 0;
            data_hit_counter = 0;
        }
        
        hal.scheduler->delay_microseconds(100);
    }
}


void AP_ExternalAHRS_SensAItion::handle_ins() {
    AP::ins().handle_external(_ins);
}

void AP_ExternalAHRS_SensAItion::handle_baro() {
#if AP_BARO_EXTERNALAHRS_ENABLED    
    AP::baro().handle_external(_baro);
#endif
}

void AP_ExternalAHRS_SensAItion::handle_compass() {
#if AP_COMPASS_EXTERNALAHRS_ENABLED
    AP::compass().handle_external(_mag);
#endif
}

void AP_ExternalAHRS_SensAItion::handle_gps() {
    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(_gps, instance);
    }
}


void AP_ExternalAHRS_SensAItion::update() {
    WITH_SEMAPHORE(sem_handle);
    if(valid_ins) handle_ins();
    if(valid_baro) handle_baro();
    if(valid_compass) handle_compass();
    if(valid_gps) handle_gps();
}; 

bool AP_ExternalAHRS_SensAItion::check_uart() {
    if (!uart) return false;

    if (!setup_complete) {
        uart->begin(baudrate);
        setup_complete = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "KEBNI: INIT. Mode:%d Baud:%u", 
            (int)_ins_mode_enabled, (unsigned)baudrate);
    }
    uint32_t n = uart->available();
    if (n == 0) return false;

    n = MIN(n, sizeof(buffer));
    ssize_t nread = uart->read(buffer, n);
    
    if (DEBUG_HEX_DUMP) {
        static uint32_t last_hex_dump = 0;
        if (AP_HAL::millis() - last_hex_dump > 500) { 
            last_hex_dump = AP_HAL::millis();
            char hex_str[64];
            uint8_t dump_len = MIN(nread, 15);
            for(uint8_t i=0; i<dump_len; i++) snprintf(&hex_str[i*3], 4, "%02X ", buffer[i]);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RAW[%d]: %s", (int)nread, hex_str);
        }
    }

    bool parsed_any = false;

    if (nread > 0) {
        parser.parse_stream(buffer, nread, [&](const AP_ExternalAHRS_SensAItion_Parser::Measurement& meas){
        
            uint32_t now_ms = AP_HAL::millis();
            uint32_t now_us = AP_HAL::micros(); 
            uint32_t t_exec_start = AP_HAL::micros(); 

            if (meas.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::UNINITIALIZED) {
                return;
            }

            parsed_any = true;
            
            if (meas.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::IMU) {
                stats_imu_rate.update_interval(now_us);
                
                if (DEBUG_TIMING && (now_ms - _last_imu_pkt_ms > 50)) {
                     GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "KEBNI: IMU Jitter! dt=%u", (unsigned)(now_ms - _last_imu_pkt_ms));
                }
                _last_imu_pkt_ms = now_ms;

                {
                    WITH_SEMAPHORE(state.sem);
                    state.accel = meas.acceleration_mss;
                    state.gyro = meas.angular_velocity_rads;
                }

                {
                    WITH_SEMAPHORE(sem_handle);
                    valid_ins = true;
                    _ins.accel = meas.acceleration_mss;
                    _ins.gyro = meas.angular_velocity_rads;
                    _ins.temperature = meas.temperature_degc;

                    valid_compass = true;
                    _mag.field = meas.magnetic_field_mgauss;

                    valid_baro = true;
                    _baro.instance = 0;
                    _baro.pressure_pa = meas.air_pressure_p;
                    _baro.temperature = meas.temperature_degc;
                    
                    //handle_ins();
                    //handle_compass();
                    handle_baro();
                }
                stats_imu_exec.update_val((double)(AP_HAL::micros() - t_exec_start));
            }
            else if (meas.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::AHRS) {
                stats_ahrs_rate.update_interval(now_us);

                _last_quat_pkt_ms = now_ms;
                WITH_SEMAPHORE(state.sem);
                state.quat = meas.orientation;
                state.have_quaternion = true;
                
                static uint32_t last_ahrs_log = 0;
                if (now_ms - last_ahrs_log > 2000) {
                     last_ahrs_log = now_ms;
                     GCS_SEND_TEXT(MAV_SEVERITY_INFO, "KEBNI: AHRS Packet Processed");
                }
                
                stats_ahrs_exec.update_val((double)(AP_HAL::micros() - t_exec_start));
            }
            else if (meas.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::INS) {
                stats_ins_rate.update_interval(now_us);

                if (now_ms - _last_ins_pkt_ms > 2000) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "KEBNI: Processing INS Packet. Fix:%d Valid:%d", 
                        (int)meas.gnss1_fix, (int)meas.sensor_valid);
                }

                _last_ins_pkt_ms = now_ms;
                _last_alignment_status = meas.alignment_status;
                _last_sensor_valid = meas.sensor_valid;
                _last_gnss1_fix = meas.gnss1_fix;
                _last_gnss2_fix = meas.gnss2_fix;
                _last_error_flags = meas.error_flags;
                _last_h_pos_quality = meas.pos_accuracy.xy().length();
                _last_v_pos_quality = meas.pos_accuracy.z;
                _last_vel_quality = meas.vel_accuracy.length();

                static uint32_t last_ins_dump = 0;
                if (now_ms - last_ins_dump > 2000) {
                    last_ins_dump = now_ms;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "INS: Lat:%ld Lon:%ld Alt:%.1f Fix:%u Val:0x%02X", 
                        (long)meas.location.lat,
                        (long)meas.location.lng,
                        (double)meas.location.alt,
                        (unsigned)_last_gnss1_fix,
                        (unsigned)_last_sensor_valid);
                }

                log_ins_status(meas);

                {
                    WITH_SEMAPHORE(sem_handle);

                    valid_gps = true;                
                    _gps.gps_week = meas.gps_week;
                    _gps.ms_tow = meas.time_itow;
                    _gps.fix_type = AP_GPS_FixType(meas.gnss1_fix);
                    _gps.satellites_in_view = meas.num_sats_gnss1;
                    _gps.horizontal_pos_accuracy = _last_h_pos_quality;
                    _gps.vertical_pos_accuracy = _last_v_pos_quality;
                    _gps.horizontal_vel_accuracy = meas.vel_accuracy.xy().length();
                    _gps.latitude = meas.location.lat;
                    _gps.longitude = meas.location.lng;
                    _gps.msl_altitude = meas.location.alt; 
                    _gps.ned_vel_north = meas.velocity_ned.x;
                    _gps.ned_vel_east = meas.velocity_ned.y;
                    _gps.ned_vel_down = meas.velocity_ned.z;

                    handle_gps();
                    
                    uint8_t instance;
                    if (!AP::gps().get_first_external_instance(instance)) {
                        static uint32_t last_nogps = 0;
                        if (now_ms > 10000 && (now_ms - last_nogps > 5000)) { 
                            last_nogps = now_ms;
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "KEBNI: FATAL - No GPS Backend (Type=21) Found!");
                        }
                    }
                }
                
                {
                    WITH_SEMAPHORE(state.sem);
                    state.location = Location(
                                              meas.location.lat,
                                              meas.location.lng,
                                              meas.location.alt,
                                              Location::AltFrame::ABSOLUTE
                                              );
                    state.velocity = meas.velocity_ned;
                    state.have_location = true;
                    state.have_velocity = true;
                    state.last_location_update_us = AP_HAL::micros();

                    if (!state.have_origin && meas.alignment_status) {
                        state.origin = Location(
                                                meas.location.lat,
                                                meas.location.lng,
                                                meas.location.alt,
                                                Location::AltFrame::ABSOLUTE
                                                );
                        state.have_origin = true;
                        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "KEBNI: Origin Set.");
                    }
                }
                
                stats_ins_exec.update_val((double)(AP_HAL::micros() - t_exec_start));
            }
        }); 
    }
    
    return parsed_any;
}

bool AP_ExternalAHRS_SensAItion::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    WITH_SEMAPHORE(state.sem);
    
    if (_ins_mode_enabled && _last_alignment_status == 1) {
        posVar = _last_h_pos_quality * pos_gate_scale;
        velVar = _last_vel_quality * vel_gate_scale;
        hgtVar = _last_v_pos_quality * hgt_gate_scale;
        tasVar = 0; //not used
        return true;
    }
    
    return false;
}

void AP_ExternalAHRS_SensAItion::log_ins_status(const AP_ExternalAHRS_SensAItion_Parser::Measurement &meas)
{
#if HAL_LOGGING_ENABLED
    if (meas.type != AP_ExternalAHRS_SensAItion_Parser::MeasurementType::INS) {
        return;
    }

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;

    {
        WITH_SEMAPHORE(state.sem);
        if (state.have_quaternion) {
            state.quat.to_euler(roll_rad, pitch_rad, yaw_rad);
        }
    }

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
