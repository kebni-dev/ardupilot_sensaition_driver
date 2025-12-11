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
#include "AP_ExternalAHRS_SensAItion_Parser.h"

class AP_ExternalAHRS_SensAItion : public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_SensAItion(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &_state);

    // Hardware Identification
    int8_t get_port() const override;
    const char* get_name() const override;

    // Health & Status Interface
    bool healthy() const override;
    bool initialised() const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    // GPS Interface
    uint8_t num_gps_sensors() const override;

    // Main Loop
    void update() override {}; 

private:
    AP_ExternalAHRS_SensAItion_Parser parser;
    AP_ExternalAHRS_SensAItion_Parser::Measurement sensor_measurement;

    // UART
    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t baudrate = 460800;
    int8_t port_num = -1;
    uint8_t buffer[AP_ExternalAHRS_SensAItion_Parser::MAX_PACKET_SIZE];

    // Threading
    bool setup_complete = false;
    void update_thread();
    bool check_uart();

    // Logging
    void log_ins_status(const AP_ExternalAHRS_SensAItion_Parser::Measurement &meas);

    // Persistent State
    uint32_t _last_imu_pkt_ms = 0;
    uint32_t _last_ins_pkt_ms = 0; // Only used in INS mode
    uint32_t _last_quat_pkt_ms = 0; // Only used in INS mode

    // Last known values from INS packet (Packet 2)
    uint8_t  _last_alignment_status = 0;
    uint8_t  _last_gnss1_fix = 0;
    uint8_t  _last_gnss2_fix = 0;
    uint8_t  _last_sensor_valid = 0;
    float    _last_h_pos_quality = 999.9f;
    float    _last_v_pos_quality = 999.9f;
    float    _last_vel_quality = 999.9f;
    uint32_t _last_error_flags = 0;

    // Configuration
    bool _ins_mode_enabled = false;
    // -----------------------------------------------------------------------
    // [CPO] Statistics Helper Struct
    // -----------------------------------------------------------------------
struct StatTracker {
        uint32_t last_us = 0;
        uint32_t count = 0;
        double sum = 0;
        double sum_sq = 0;
        float max_val = 0; 

        // Mode A: Interval (Jitter)
        void update_interval(uint32_t now_us) {
            if (last_us != 0) {
                double dt = (double)(now_us - last_us);
                update_val(dt);
            }
            last_us = now_us;
        }

        // Mode B: Value (Execution Time)
        void update_val(double val) {
            sum += val;
            sum_sq += (val * val);
            if (val > max_val) max_val = (float)val;
            count++;
        }

        void get_stats(float &mean, float &std_dev, float &peak) {
            if (count < 2) {
                mean = 0; std_dev = 0; peak = 0; return;
            }
            mean = (float)(sum / count);
            double variance = (sum_sq - (sum * sum) / count) / (count - 1);
            std_dev = (float)sqrt(MAX(variance, 0.0));
            peak = max_val;
        }

        void reset() {
            count = 0; sum = 0; sum_sq = 0; max_val = 0;
        }
    };

    // --- Performance Trackers ---
    
    // Loop Statistics
    StatTracker stats_loop_rate;
    
    // [CPO] Split Execution Analysis
    StatTracker stats_exec_idle; // Loop time when NO packet found
    StatTracker stats_exec_busy; // Loop time when PACKET found

    uint32_t loop_counter = 0;
    uint32_t data_hit_counter = 0;

    // Data Stream Interval (Jitter)
    StatTracker stats_imu_rate;
    StatTracker stats_ahrs_rate;
    StatTracker stats_ins_rate;

    // Processing Cost per Packet (Execution Time)
    StatTracker stats_imu_exec;
    StatTracker stats_ahrs_exec;
    StatTracker stats_ins_exec;

    uint32_t last_stat_report_ms = 0;
};

#endif  // AP_EXTERNAL_AHRS_SENSAITION_ENABLED