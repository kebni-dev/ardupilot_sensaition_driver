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
  Simulate SensAItion serial IMU/AHRS device
  Converts SITL flight dynamics to SensAItion protocol packets
*/

#include "SIM_SensAItion.h"
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>

using namespace SITL;

SensAItion::SensAItion() :
    SerialDevice::SerialDevice()
{
    last_imu_pkt_us = 0;
    last_ahrs_pkt_us = 0;
}

// SensAItion packet data structures (header and checksum sent separately)
// IMU packet: 36 bytes data (accel 12B + gyro 12B + temp 2B + mag 6B + baro 4B)
struct PACKED SensAItion_IMU_packet {
    int32_t accel_x;        // Bytes 0-3: µg units (1e-6 g)
    int32_t accel_y;        // Bytes 4-7: µg units
    int32_t accel_z;        // Bytes 8-11: µg units
    int32_t gyro_x;         // Bytes 12-15: µdeg/s units (1e-6 deg/s)
    int32_t gyro_y;         // Bytes 16-19: µdeg/s units
    int32_t gyro_z;         // Bytes 20-23: µdeg/s units
    int16_t temperature;    // Bytes 24-25: special formula (temp_c - 20) / 0.008
    int16_t mag_x;          // Bytes 26-27: mGauss units
    int16_t mag_y;          // Bytes 28-29: mGauss units
    int16_t mag_z;          // Bytes 30-31: mGauss units
    int32_t baro;           // Bytes 32-35: 0.1 Pa units
};

// AHRS packet: 52 bytes data (36B IMU + 16B quaternion)
struct PACKED SensAItion_AHRS_packet {
    // IMU data (same as IMU packet, 36 bytes)
    int32_t accel_x;        // Bytes 0-3: µg units (1e-6 g)
    int32_t accel_y;        // Bytes 4-7: µg units
    int32_t accel_z;        // Bytes 8-11: µg units
    int32_t gyro_x;         // Bytes 12-15: µdeg/s units (1e-6 deg/s)
    int32_t gyro_y;         // Bytes 16-19: µdeg/s units
    int32_t gyro_z;         // Bytes 20-23: µdeg/s units
    int16_t temperature;    // Bytes 24-25: special formula
    int16_t mag_x;          // Bytes 26-27: mGauss units
    int16_t mag_y;          // Bytes 28-29: mGauss units
    int16_t mag_z;          // Bytes 30-31: mGauss units
    int32_t baro;           // Bytes 32-35: 0.1 Pa units
    // AHRS quaternion data (16 bytes)
    int32_t quat_w;         // Bytes 36-39: 1e-6 scale
    int32_t quat_x;         // Bytes 40-43: 1e-6 scale
    int32_t quat_y;         // Bytes 44-47: 1e-6 scale
    int32_t quat_z;         // Bytes 48-51: 1e-6 scale
};


uint8_t SensAItion::calculate_xor_checksum(const uint8_t* data, uint16_t length)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Main update loop: Generate packets at configured rates
void SensAItion::update(void)
{
    const uint32_t now_us = AP_HAL::micros();

    // Send IMU packets at 1000Hz (1ms intervals)
    // Used for IMU-only mode testing
    if (now_us - last_imu_pkt_us >= 1000) {
        send_imu_packet();
        last_imu_pkt_us = now_us;
    }

    // Send AHRS packets at 500Hz (2ms intervals)
    // Used for AHRS mode testing
    if (now_us - last_ahrs_pkt_us >= 2000) {
        send_ahrs_packet();
        last_ahrs_pkt_us = now_us;
    }
}

void SensAItion::send_imu_packet(void)
{
    const auto &fdm = _sitl->state;

    struct SensAItion_IMU_packet pkt {};

    // Convert from SI units to SensAItion units
    // Acceleration: m/s² to µg (1e-6 g)
    const float gravity = 9.80665f;
    pkt.accel_x = (int32_t)(fdm.xAccel / gravity * 1e6);
    pkt.accel_y = (int32_t)(fdm.yAccel / gravity * 1e6);
    pkt.accel_z = (int32_t)(fdm.zAccel / gravity * 1e6);

    // Angular rates: rad/s to µdeg/s (1e-6 deg/s)
    pkt.gyro_x = (int32_t)(degrees(fdm.rollRate) * 1e6);
    pkt.gyro_y = (int32_t)(degrees(fdm.pitchRate) * 1e6);
    pkt.gyro_z = (int32_t)(degrees(fdm.yawRate) * 1e6);

    // Temperature: special conversion formula
    float temp_c = 25.0f;
    pkt.temperature = (int16_t)((temp_c - 20.0f) / 0.008f);

    // Magnetometer: Gauss to mGauss
    pkt.mag_x = (int16_t)(fdm.bodyMagField.x * 1000);
    pkt.mag_y = (int16_t)(fdm.bodyMagField.y * 1000);
    pkt.mag_z = (int16_t)(fdm.bodyMagField.z * 1000);

    // Barometer: Calculate pressure from altitude (Pa to 0.1 Pa units)
    float altitude_m = fdm.altitude;
    float pressure_pa = 101325.0f * powf(1.0f - 2.25577e-5f * altitude_m, 5.25588f);
    pkt.baro = (int32_t)(pressure_pa * 10.0f);

    // Periodic status output to verify simulator operation
    static uint32_t imu_count = 0;
    if (++imu_count % 1000 == 0) {
        ::printf("SensAItion: IMU packet #%u sent - accel_x=%d µg, gyro_x=%d µdeg/s\n",
                 imu_count, pkt.accel_x, pkt.gyro_x);
    }

    // Send header
    const uint8_t header = 0xFA;
    write_to_autopilot((const char *)&header, 1);

    // Send packet data
    write_to_autopilot((const char *)&pkt, sizeof(pkt));

    // Calculate and send checksum
    uint8_t checksum = calculate_xor_checksum((const uint8_t *)&pkt, sizeof(pkt));
    write_to_autopilot((const char *)&checksum, 1);
}

void SensAItion::send_ahrs_packet(void)
{
    const auto &fdm = _sitl->state;

    struct SensAItion_AHRS_packet pkt {};

    // IMU data (same as IMU packet)
    // Acceleration: m/s² to µg (1e-6 g)
    const float gravity = 9.80665f;
    pkt.accel_x = (int32_t)(fdm.xAccel / gravity * 1e6);
    pkt.accel_y = (int32_t)(fdm.yAccel / gravity * 1e6);
    pkt.accel_z = (int32_t)(fdm.zAccel / gravity * 1e6);

    // Angular rates: rad/s to µdeg/s (1e-6 deg/s)
    pkt.gyro_x = (int32_t)(degrees(fdm.rollRate) * 1e6);
    pkt.gyro_y = (int32_t)(degrees(fdm.pitchRate) * 1e6);
    pkt.gyro_z = (int32_t)(degrees(fdm.yawRate) * 1e6);

    // Temperature: special conversion formula
    float temp_c = 25.0f;
    pkt.temperature = (int16_t)((temp_c - 20.0f) / 0.008f);

    // Magnetometer: Gauss to mGauss
    pkt.mag_x = (int16_t)(fdm.bodyMagField.x * 1000);
    pkt.mag_y = (int16_t)(fdm.bodyMagField.y * 1000);
    pkt.mag_z = (int16_t)(fdm.bodyMagField.z * 1000);

    // Barometer: Calculate pressure from altitude (Pa to 0.1 Pa units)
    float altitude_m = fdm.altitude;
    float pressure_pa = 101325.0f * powf(1.0f - 2.25577e-5f * altitude_m, 5.25588f);
    pkt.baro = (int32_t)(pressure_pa * 10.0f);

    // Quaternion: Convert from euler angles (SITL fdm provides roll/pitch/yaw)
    // Create quaternion from euler angles (roll, pitch, yaw)
    float roll = fdm.rollDeg * DEG_TO_RAD;
    float pitch = fdm.pitchDeg * DEG_TO_RAD;
    float yaw = fdm.yawDeg * DEG_TO_RAD;

    // Quaternion from euler angles
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    float qw = cr * cp * cy + sr * sp * sy;
    float qx = sr * cp * cy - cr * sp * sy;
    float qy = cr * sp * cy + sr * cp * sy;
    float qz = cr * cp * sy - sr * sp * cy;

    // Convert to 1e-6 scale
    pkt.quat_w = (int32_t)(qw * 1e6f);
    pkt.quat_x = (int32_t)(qx * 1e6f);
    pkt.quat_y = (int32_t)(qy * 1e6f);
    pkt.quat_z = (int32_t)(qz * 1e6f);

    // Periodic status output
    static uint32_t ahrs_count = 0;
    if (++ahrs_count % 500 == 0) {
        ::printf("SensAItion: AHRS packet #%u sent\n", ahrs_count);
    }

    // Send header
    const uint8_t header = 0xFA;
    write_to_autopilot((const char *)&header, 1);

    // Send packet data
    write_to_autopilot((const char *)&pkt, sizeof(pkt));

    // Calculate and send checksum
    uint8_t checksum = calculate_xor_checksum((const uint8_t *)&pkt, sizeof(pkt));
    write_to_autopilot((const char *)&checksum, 1);
}