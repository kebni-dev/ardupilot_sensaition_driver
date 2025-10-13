#include <AP_gtest.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS_SensAItion_Parser.h>

namespace {
constexpr float UG_PER_MSS = 1e6f / 9.80665f; // ug per m/s^2
constexpr float UDEGS_PER_RADS = 1e6f * 180.0f / 3.1415926f; // udeg/s per rad/s
constexpr float MHPA_PER_PA = 1e3f / 100.0f; // mhPa per Pa
const float MEASUREMENT_TOLERANCE = 1e-3f;
}

static void fill_with_uint32_in_big_endian_order(uint8_t* data, size_t& location, const uint32_t value)
{
    data[location++] = (value & 0xFF000000) >> 24;
    data[location++] = (value & 0x00FF0000) >> 16;
    data[location++] = (value & 0x0000FF00) >> 8;
    data[location++] = (value & 0x000000FF);
}

static void fill_with_uint16_in_big_endian_order(uint8_t* data, size_t& location, const uint16_t value)
{
    data[location++] = (value & 0xFF00) >> 8;
    data[location++] = (value & 0x00FF);
}

/*
Fill a simulated SensAItion binary packet with data corresponding to the given measurement struct,
including header byte and checksum.

data: Pointer to the buffer to fill
data_length: Input the maximum length of the data, will be reset to the actual packet length
measurement: Measurement values to pack into the packet
*/
static void fill_simulated_packet(uint8_t* data, size_t& data_length,
    const AP_ExternalAHRS_SensAItion_Parser::Measurement measurement)
{
    assert(measurement.type != AP_ExternalAHRS_SensAItion_Parser::MeasurementType::UNINITIALIZED);

    // IMU: 1 (header) + 7*4 + 4*2 + 1 (checksum) = 38 bytes
    // AHRS: IMU + 4*4 = 54 bytes
    const size_t packet_length = (measurement.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::IMU ? 38 : 54);
    assert(data_length >= packet_length);
    
    size_t byte_count = 0;
    data[byte_count++] = 0xFA; // Header

    // Accelerometer (raw data is in ug)
    fill_with_uint32_in_big_endian_order(data, byte_count, measurement.acceleration_mss.x * UG_PER_MSS);
    fill_with_uint32_in_big_endian_order(data, byte_count, measurement.acceleration_mss.y * UG_PER_MSS);
    fill_with_uint32_in_big_endian_order(data, byte_count, measurement.acceleration_mss.z * UG_PER_MSS);

    // Gyro (raw data is in udeg/s)
    fill_with_uint32_in_big_endian_order(data, byte_count, measurement.angular_velocity_rads.x * UDEGS_PER_RADS);
    fill_with_uint32_in_big_endian_order(data, byte_count, measurement.angular_velocity_rads.y * UDEGS_PER_RADS);
    fill_with_uint32_in_big_endian_order(data, byte_count, measurement.angular_velocity_rads.z * UDEGS_PER_RADS);

    // Temperature (conversion formula from SensAItion user manual)
    const uint16_t temp_raw = (measurement.temperature_degc - 20.0f) / 80.0f * 1e4f;
    fill_with_uint16_in_big_endian_order(data, byte_count, temp_raw);

    // Magnetometer (raw data is mgauss, limited to 16 bits)
    fill_with_uint16_in_big_endian_order(data, byte_count, measurement.magnetic_field_mgauss.x);
    fill_with_uint16_in_big_endian_order(data, byte_count, measurement.magnetic_field_mgauss.y);
    fill_with_uint16_in_big_endian_order(data, byte_count, measurement.magnetic_field_mgauss.z);
    
    // Barometer (raw data is mhPa)
    fill_with_uint32_in_big_endian_order(data, byte_count, measurement.air_pressure_p * MHPA_PER_PA);
    
    if (measurement.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::AHRS) {
        // Quaternion (raw data is scaled up by 1e6)
        fill_with_uint32_in_big_endian_order(data, byte_count, measurement.orientation.q1 * 1e6); // scalar 
        fill_with_uint32_in_big_endian_order(data, byte_count, measurement.orientation.q2 * 1e6); // q_i
        fill_with_uint32_in_big_endian_order(data, byte_count, measurement.orientation.q3 * 1e6); // q_j
        fill_with_uint32_in_big_endian_order(data, byte_count, measurement.orientation.q4 * 1e6); // q_k
    }

    // Compute checksum, not including the first header byte
    uint8_t checksum = 0;
    for (size_t i = 1; i < byte_count; ++i) {
        checksum = checksum ^ data[i];
    }
    data[byte_count++] = checksum;

    assert(byte_count == packet_length);
    data_length = packet_length;
}

TEST(SensAItionParser, ValidIMUPacket)
{
    AP_ExternalAHRS_SensAItion_Parser parser(AP_ExternalAHRS_SensAItion_Parser::ConfigMode::CONFIG_MODE_IMU);

    AP_ExternalAHRS_SensAItion_Parser::Measurement in;
    in.acceleration_mss = Vector3f(0.012f, 0.187f, 9.7f);
    in.angular_velocity_rads = Vector3f(0.02f, -0.03f, 0.07f);
    in.temperature_degc = 25.0f;
    in.magnetic_field_mgauss = Vector3f(-37.0f, 12.7f, 478.0f);
    in.air_pressure_p = 101300;
    in.type = AP_ExternalAHRS_SensAItion_Parser::MeasurementType::IMU;

    uint8_t packet[38];
    size_t packet_length = 38;
    fill_simulated_packet(packet, packet_length, in);

    AP_ExternalAHRS_SensAItion_Parser::Measurement out;
    parser.parse_bytes(packet, packet_length, out);

    EXPECT_TRUE(out.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::IMU);
    EXPECT_LT((out.acceleration_mss - in.acceleration_mss).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT((out.angular_velocity_rads - in.angular_velocity_rads).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT(abs(out.temperature_degc - in.temperature_degc), MEASUREMENT_TOLERANCE);
    EXPECT_LT((out.magnetic_field_mgauss - in.magnetic_field_mgauss).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT(abs(out.air_pressure_p - in.air_pressure_p), MEASUREMENT_TOLERANCE);
}

/*

TEST(SensAItionParser, ValidAHRSPacket)
{
    AP_ExternalAHRS_SensAItion_Parser parser(AP_ExternalAHRS_SensAItion_Parser::ConfigMode::CONFIG_MODE_AHRS);

    AP_ExternalAHRS_SensAItion_Parser::Measurement in;
    in.acceleration_mss = Vector3f(0.012f, 0.187f, 9.7f);
    in.angular_velocity_rads = Vector3f(0.02f, -0.03f, 0.07f);
    in.temperature_degc = 25.0f;
    in.magnetic_field_mgauss = Vector3f(-37.0f, 12.7f, 478.0f);
    in.air_pressure_p = 101300;
    in.orientation.from_euler(0.12f, 5.67f, -6.62f);
    in.type = AP_ExternalAHRS_SensAItion_Parser::MeasurementType::IMU;

    uint8_t packet[38];
    size_t packet_length = 38;
    fill_simulated_packet(packet, packet_length, in);

    AP_ExternalAHRS_SensAItion_Parser::Measurement out;
    parser.parse_bytes(packet, packet_length, out);

    EXPECT_TRUE(out.type == AP_ExternalAHRS_SensAItion_Parser::MeasurementType::IMU);
    EXPECT_LT((out.acceleration_mss - in.acceleration_mss).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT((out.angular_velocity_rads - in.angular_velocity_rads).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT(abs(out.temperature_degc - in.temperature_degc), MEASUREMENT_TOLERANCE);
    EXPECT_LT((out.magnetic_field_mgauss - in.magnetic_field_mgauss).length(), MEASUREMENT_TOLERANCE);
    EXPECT_LT(abs(out.air_pressure_p - in.air_pressure_p), MEASUREMENT_TOLERANCE);
    
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_LT(abs(out.orientation[i] - in.orientation[i]), MEASUREMENT_TOLERANCE);
    }
}

TEST(SensAItionParser, InvalidChecksumIMU)
{
    AP_ExternalAHRS_SensAItion_Parser parser;

    // Invalid packet: correct size but wrong checksum
    uint8_t invalid_packet[38];
    invalid_packet[0] = 0xFA;  // Header

    for (int i = 1; i <= 36; i++) {
        invalid_packet[i] = i;
    }

    invalid_packet[37] = 0x00;  // Wrong checksum

    const uint8_t* packet_out;
    size_t packet_size;

    bool result = parser.parse_bytes(invalid_packet, sizeof(invalid_packet), packet_out, packet_size);

    EXPECT_FALSE(result);
    EXPECT_EQ(parser.get_valid_packets(), 0U);
    EXPECT_EQ(parser.get_parse_errors(), 1U);
}

TEST(SensAItionParser, ByteByByteParsing)
{
    AP_ExternalAHRS_SensAItion_Parser parser;

    // Valid IMU packet
    uint8_t valid_packet[38];
    valid_packet[0] = 0xFA;

    for (int i = 1; i <= 36; i++) {
        valid_packet[i] = i;
    }

    uint8_t checksum = 0;
    for (int i = 1; i <= 36; i++) {
        checksum ^= valid_packet[i];
    }
    valid_packet[37] = checksum;

    // Feed bytes one at a time
    const uint8_t* packet_out;
    size_t packet_size;
    bool result = false;

    for (size_t i = 0; i < sizeof(valid_packet); i++) {
        result = parser.parse_bytes(&valid_packet[i], 1, packet_out, packet_size);
    }

    // Should succeed on last byte
    EXPECT_TRUE(result);
    EXPECT_EQ(packet_size, 36U);
    EXPECT_EQ(parser.get_valid_packets(), 1U);
}

TEST(SensAItionParser, NoiseBeforeHeader)
{
    AP_ExternalAHRS_SensAItion_Parser parser;

    // Noise bytes followed by valid packet
    uint8_t buffer[45];

    // Noise
    buffer[0] = 0x12;
    buffer[1] = 0x34;
    buffer[2] = 0x56;
    buffer[3] = 0x78;
    buffer[4] = 0x9A;
    buffer[5] = 0xBC;
    buffer[6] = 0xDE;

    // Valid packet starts at index 7
    buffer[7] = 0xFA;  // Header
    for (int i = 1; i <= 36; i++) {
        buffer[7 + i] = i;
    }

    uint8_t checksum = 0;
    for (int i = 1; i <= 36; i++) {
        checksum ^= buffer[7 + i];
    }
    buffer[44] = checksum;

    const uint8_t* packet_out;
    size_t packet_size;

    bool result = parser.parse_bytes(buffer, sizeof(buffer), packet_out, packet_size);

    // Should find valid packet despite noise
    EXPECT_TRUE(result);
    EXPECT_EQ(packet_size, 36U);
    EXPECT_EQ(parser.get_valid_packets(), 1U);
}

TEST(SensAItionParser, MixedValidInvalidPackets)
{
    AP_ExternalAHRS_SensAItion_Parser parser;

    // First packet: valid
    uint8_t packet1[38];
    packet1[0] = 0xFA;
    for (int i = 1; i <= 36; i++) {
        packet1[i] = i;
    }
    uint8_t checksum1 = 0;
    for (int i = 1; i <= 36; i++) {
        checksum1 ^= packet1[i];
    }
    packet1[37] = checksum1;

    // Second packet: invalid checksum
    uint8_t packet2[38];
    packet2[0] = 0xFA;
    for (int i = 1; i <= 36; i++) {
        packet2[i] = i + 10;
    }
    packet2[37] = 0x00;  // Wrong checksum

    const uint8_t* packet_out;
    size_t packet_size;

    // Parse first packet
    bool result1 = parser.parse_bytes(packet1, sizeof(packet1), packet_out, packet_size);
    EXPECT_TRUE(result1);

    // Parse second packet
    bool result2 = parser.parse_bytes(packet2, sizeof(packet2), packet_out, packet_size);
    EXPECT_FALSE(result2);

    // Check counters
    EXPECT_EQ(parser.get_valid_packets(), 1U);
    EXPECT_EQ(parser.get_parse_errors(), 1U);
}

TEST(SensAItionParser, ValidAHRSPacket)
{
    AP_ExternalAHRS_SensAItion_Parser parser(AP_ExternalAHRS_SensAItion_Parser::ConfigMode::CONFIG_MODE_AHRS);

    // Valid AHRS packet: Header + 52 data bytes + checksum = 54 total
    uint8_t valid_packet[54];
    valid_packet[0] = 0xFA;  // Header

    // Fill with test data (13 sensors × 4 bytes)
    for (int i = 1; i <= 52; i++) {
        valid_packet[i] = i;
    }

    // Calculate XOR checksum
    uint8_t checksum = 0;
    for (int i = 1; i <= 52; i++) {
        checksum ^= valid_packet[i];
    }
    valid_packet[53] = checksum;

    const uint8_t* packet_out;
    size_t packet_size;

    bool result = parser.parse_bytes(valid_packet, sizeof(valid_packet), packet_out, packet_size);

    EXPECT_TRUE(result);
    EXPECT_EQ(packet_size, 52U);
    EXPECT_EQ(parser.get_valid_packets(), 1U);
    EXPECT_EQ(parser.get_parse_errors(), 0U);
}

TEST(SensAItionParser, InvalidChecksumAHRS)
{
    AP_ExternalAHRS_SensAItion_Parser parser(AP_ExternalAHRS_SensAItion_Parser::ConfigMode::CONFIG_MODE_AHRS);

    // Invalid AHRS packet: correct size but wrong checksum
    uint8_t invalid_packet[54];
    invalid_packet[0] = 0xFA;  // Header

    for (int i = 1; i <= 52; i++) {
        invalid_packet[i] = i;
    }

    invalid_packet[53] = 0x00;  // Wrong checksum

    const uint8_t* packet_out;
    size_t packet_size;

    bool result = parser.parse_bytes(invalid_packet, sizeof(invalid_packet), packet_out, packet_size);

    EXPECT_FALSE(result);
    EXPECT_EQ(parser.get_valid_packets(), 0U);
    EXPECT_EQ(parser.get_parse_errors(), 1U);
}
*/

AP_GTEST_MAIN()
