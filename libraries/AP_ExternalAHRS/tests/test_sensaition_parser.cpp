
#include <AP_gtest.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS_SensAItion_Parser.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {
// Units definitions Kebni
constexpr float UG_PER_MSS = 1e6f / 9.80665f;
constexpr float UDEGS_PER_RADS = 1e6f * 180.0f / 3.1415926f;
constexpr float MHPA_PER_PA = 1e3f / 100.0f;
const float MEASUREMENT_TOLERANCE = 1e-3f;

using Parser = AP_ExternalAHRS_SensAItion_Parser;
}

// --- HELPER FUNCTIONS ---
static void fill_be32(uint8_t* data, size_t& loc, int32_t val) {
    data[loc++] = (val >> 24) & 0xFF;
    data[loc++] = (val >> 16) & 0xFF;
    data[loc++] = (val >> 8) & 0xFF;
    data[loc++] = val & 0xFF;
}

static void fill_be16(uint8_t* data, size_t& loc, int16_t val) {
    data[loc++] = (val >> 8) & 0xFF;
    data[loc++] = val & 0xFF;
}

static void fill_u8(uint8_t* data, size_t& loc, uint8_t val) {
    data[loc++] = val;
}

// --- PACKET GENERATOR ---
static void fill_simulated_packet(uint8_t* data, size_t& data_length,
                                  const Parser::Measurement& m,
                                  Parser::ConfigMode mode)
{
    size_t idx = 0;
    
    // 1. Header
    data[idx++] = 0xFA;

    // 2. ID (Only for Interleaved Mode)
    if (mode == Parser::ConfigMode::INTERLEAVED_INS) {
        switch (m.type) {
            case Parser::MeasurementType::IMU:  data[idx++] = 0x00; break;
            case Parser::MeasurementType::AHRS: data[idx++] = 0x01; break;
            case Parser::MeasurementType::INS:  data[idx++] = 0x02; break;
            default: break;
        }
    }

    // 3. Payload Generation
    if (m.type == Parser::MeasurementType::IMU) {
        fill_be32(data, idx, m.acceleration_mss.x * UG_PER_MSS);
        fill_be32(data, idx, m.acceleration_mss.y * UG_PER_MSS);
        fill_be32(data, idx, m.acceleration_mss.z * UG_PER_MSS);
        fill_be32(data, idx, m.angular_velocity_rads.x * UDEGS_PER_RADS);
        fill_be32(data, idx, m.angular_velocity_rads.y * UDEGS_PER_RADS);
        fill_be32(data, idx, m.angular_velocity_rads.z * UDEGS_PER_RADS);
        fill_be16(data, idx, (int16_t)((m.temperature_degc - 20.0f) / 0.008f));
        fill_be16(data, idx, m.magnetic_field_mgauss.x);
        fill_be16(data, idx, m.magnetic_field_mgauss.y);
        fill_be16(data, idx, m.magnetic_field_mgauss.z);
        fill_be32(data, idx, m.air_pressure_p * MHPA_PER_PA);

    } else if (m.type == Parser::MeasurementType::AHRS) {
        fill_be32(data, idx, m.orientation.q1 * 1e6);
        fill_be32(data, idx, m.orientation.q2 * 1e6);
        fill_be32(data, idx, m.orientation.q3 * 1e6);
        fill_be32(data, idx, m.orientation.q4 * 1e6);

} else if (m.type == Parser::MeasurementType::INS) {
        // --- NEW 50-BYTE LAYOUT (STRICT ORDER) ---
        
        // Bytes 0-3: Num Sats (Split logic: Byte 0=GNSS1, Byte 2=GNSS2)
        // We use fake values here: 12 sats for GNSS1, 10 for GNSS2
        fill_u8(data, idx, 12); // Byte 0: GNSS1 Count
        fill_u8(data, idx, 0);  // Byte 1: Padding
        fill_u8(data, idx, 10); // Byte 2: GNSS2 Count
        fill_u8(data, idx, 0);  // Byte 3: Padding

        // Bytes 4-7: Error Flags
        fill_be32(data, idx, m.error_flags);

        // Byte 8: Sensor Valid
        fill_u8(data, idx, m.sensor_valid);

        // Bytes 9-12: Latitude
        fill_be32(data, idx, m.location.lat);

        // Bytes 13-16: Longitude
        fill_be32(data, idx, m.location.lng);

        // Bytes 17-28: Velocity N, E, D
        fill_be32(data, idx, m.velocity_ned.x * 1000); // N (mm/s)
        fill_be32(data, idx, m.velocity_ned.y * 1000); // E
        fill_be32(data, idx, m.velocity_ned.z * 1000); // D

        // Bytes 29-32: Altitude MSL
        fill_be32(data, idx, m.location.alt * 10); // cm -> mm

        // Byte 33: Alignment Status
        fill_u8(data, idx, m.alignment_status);

        // Bytes 34-37: Time iTOW
        fill_be32(data, idx, 123456789); // Fake Timestamp

        // Bytes 38-41: GNSS Fix (16bit GNSS1, 16bit GNSS2)
        fill_be16(data, idx, m.gnss1_fix);
        fill_be16(data, idx, m.gnss2_fix);

        // Bytes 42-45: Pos Accuracy
        fill_be32(data, idx, m.pos_accuracy_horiz * 1000);

        // Bytes 46-49: Vel Accuracy
        fill_be32(data, idx, m.vel_accuracy * 1000);
    }

    // 4. CRC
    uint8_t checksum = 0;
    for (size_t i = 1; i < idx; ++i) checksum ^= data[i];
    data[idx++] = checksum;

    data_length = idx;
}

// --- TEST DATA FACTORY ---
static Parser::Measurement default_measurement(Parser::MeasurementType type)
{
    Parser::Measurement in;
    in.type = type;
    in.acceleration_mss = Vector3f(0.01f, 0.02f, 9.81f);
    in.angular_velocity_rads = Vector3f(0.01f, -0.02f, 0.03f);
    in.temperature_degc = 25.0f;
    in.magnetic_field_mgauss = Vector3f(-10.0f, 20.0f, 500.0f);
    in.air_pressure_p = 101325;
    in.alignment_status = 1;
    in.gnss1_fix = 3;
    in.gnss2_fix = 3;
    in.location.lat = 593293230; 
    in.location.lng = 180685810; 
    in.location.alt = 5000;      
    in.velocity_ned = Vector3f(0.5f, -0.2f, 0.1f);
    in.pos_accuracy_horiz = 0.5f;
    in.vel_accuracy = 0.1f;
    in.error_flags = 0;
    in.sensor_valid = 0xFF;
    return in;
}

// ---------------------------------------------------------------------------
// LEGACY MODE TESTS (IMU ONLY)
// ---------------------------------------------------------------------------

TEST(SensAItionParser, Legacy_IMU_HappyPath)
{
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t buffer[100];
    size_t len = 100;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::IMU);

    EXPECT_EQ(len, 38u); 

    Parser::Measurement out;
    uint32_t start_valid = parser.get_valid_packets();

    parser.parse_bytes(buffer, len, out);

    EXPECT_EQ(parser.get_valid_packets(), start_valid + 1);
    EXPECT_EQ(out.type, Parser::MeasurementType::IMU);
    EXPECT_NEAR(out.acceleration_mss.z, 9.81f, 0.01f);
}

TEST(SensAItionParser, Legacy_RejectsInvalidChecksum)
{
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t buffer[100];
    size_t len = 100;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::IMU);

    // Corrupt Checksum
    buffer[len - 1] += 1; 

    Parser::Measurement out;
    uint32_t start_valid = parser.get_valid_packets();
    uint32_t start_errors = parser.get_parse_errors();

    parser.parse_bytes(buffer, len, out);

    EXPECT_EQ(parser.get_valid_packets(), start_valid);
    EXPECT_GT(parser.get_parse_errors(), start_errors);
}

TEST(SensAItionParser, Legacy_RejectsTooSmallBuffer)
{
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t buffer[100];
    size_t len = 100;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::IMU);

    Parser::Measurement out;
    uint32_t start_valid = parser.get_valid_packets();

    // Feed partial packet (len - 5 bytes)
    parser.parse_bytes(buffer, len - 5, out);

    EXPECT_EQ(parser.get_valid_packets(), start_valid);
}

TEST(SensAItionParser, Legacy_ValidPacketsCount)
{
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t buffer[100];
    size_t len = 100;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::IMU);

    Parser::Measurement out;
    uint32_t start_valid = parser.get_valid_packets();

    // Feed packet 5 times
    for (int i=0; i<5; i++) {
        parser.parse_bytes(buffer, len, out);
    }

    EXPECT_EQ(parser.get_valid_packets(), start_valid + 5);
}

// --- LEGACY TORTURE SUITE ---

TEST(SensAItionParser, Legacy_FalseHeaderInPayload)
{
    // PROVES: Parser robustness against 0xFA in data
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    
    uint8_t packet[38];
    size_t len = 38;
    fill_simulated_packet(packet, len, in, Parser::ConfigMode::IMU);

    // INJECT 0xFA (Byte index 5)
    packet[5] = 0xFA;
    // REPAIR CRC
    uint8_t checksum = 0;
    for (size_t i = 1; i < len - 1; ++i) checksum ^= packet[i];
    packet[len - 1] = checksum;

    Parser::Measurement out;
    uint32_t start_valid = parser.get_valid_packets();
    
    // FEED BYTES ONE BY ONE
    for (size_t i = 0; i < len; i++) {
        parser.parse_bytes(&packet[i], 1, out);
        
        if (i < len - 1) {
            EXPECT_EQ(parser.get_valid_packets(), start_valid) 
                << "Parser triggered prematurely at index " << i;
        }
    }
    
    EXPECT_EQ(parser.get_valid_packets(), start_valid + 1)
        << "Parser failed to accept valid packet with internal 0xFA";
}

TEST(SensAItionParser, Legacy_FragmentedHeaderRecovery)
{
    // PROVES: Recovery after false start
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    uint8_t valid_packet[38];
    size_t len = 38;
    fill_simulated_packet(valid_packet, len, in, Parser::ConfigMode::IMU);

    uint8_t stream[120];
    size_t slen = 0;

    // 1. False Start (FA 00 ...) -> Traps parser
    stream[slen++] = 0xFA; 
    stream[slen++] = 0x00; 
    
    // 2. Valid Packet 1 (Sacrificed due to overlap)
    memcpy(&stream[slen], valid_packet, 38);
    slen += 38;

    // 3. Valid Packet 2 (Must be recovered)
    memcpy(&stream[slen], valid_packet, 38);
    slen += 38;

    Parser::Measurement out;
    uint32_t start_valid = parser.get_valid_packets();

    for (size_t i = 0; i < slen; i++) {
        parser.parse_bytes(&stream[i], 1, out);
    }

    uint32_t total_valid = parser.get_valid_packets() - start_valid;
    EXPECT_GE(total_valid, 1u) << "Parser died after false header";
}

// ---------------------------------------------------------------------------
// INTERLEAVED MODE TESTS
// ---------------------------------------------------------------------------

TEST(SensAItionParser, Interleaved_IMU_HappyPath)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    in.acceleration_mss.x = 2.5f; 
    uint8_t buffer[64];
    size_t len = 64;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::INTERLEAVED_INS);

    EXPECT_EQ(len, 39u); 
    Parser::Measurement out;
    uint32_t start_valid = parser.get_valid_packets();

    parser.parse_bytes(buffer, len, out);

    EXPECT_EQ(parser.get_valid_packets(), start_valid + 1);
    EXPECT_EQ(out.type, Parser::MeasurementType::IMU);
    EXPECT_NEAR(out.acceleration_mss.x, 2.5f, 0.01f);
}

TEST(SensAItionParser, Interleaved_INS_HappyPath)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto in = default_measurement(Parser::MeasurementType::INS);
    in.location.alt = 12300; 
    uint8_t buffer[100];
    size_t len = 100;
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::INTERLEAVED_INS);

    EXPECT_EQ(len, 53u);
    Parser::Measurement out;
    uint32_t start_valid = parser.get_valid_packets();

    parser.parse_bytes(buffer, len, out);

    EXPECT_EQ(parser.get_valid_packets(), start_valid + 1);
    EXPECT_EQ(out.type, Parser::MeasurementType::INS);
    EXPECT_EQ(out.location.alt, 12300);
}

TEST(SensAItionParser, Interleaved_InvalidID)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    Parser::Measurement out;
    
    // FA followed by invalid ID 99
    uint8_t bad_packet[] = { 0xFA, 0x99, 0x00, 0x00 }; 
    uint32_t start_valid = parser.get_valid_packets();
    uint32_t start_errors = parser.get_parse_errors();

    parser.parse_bytes(bad_packet, 4, out);
    
    EXPECT_EQ(parser.get_valid_packets(), start_valid);
    EXPECT_GT(parser.get_parse_errors(), start_errors);
}

// ---------------------------------------------------------------------------
// MISSING TESTS FROM SPEC CHAPTER 7.1
// ---------------------------------------------------------------------------

// REQ: "Verify seamless transition between Packet 0, Packet 1, and Packet 2" [Spec 7.1.A]
TEST(SensAItionParser, Interleaved_MixedStream_Transitions)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    
    // Create one of each measurement type
    auto m_imu = default_measurement(Parser::MeasurementType::IMU);
    auto m_ahrs = default_measurement(Parser::MeasurementType::AHRS);
    auto m_ins = default_measurement(Parser::MeasurementType::INS);

    // Build a continuous stream: [IMU][AHRS][INS]
    uint8_t stream[200];
    size_t len = 0;
    size_t part_len = 0;

    // Append IMU (39 bytes)
    part_len = 200 - len;
    fill_simulated_packet(&stream[len], part_len, m_imu, Parser::ConfigMode::INTERLEAVED_INS);
    len += part_len; // 39

    // Append AHRS (19 bytes)
    part_len = 200 - len;
    fill_simulated_packet(&stream[len], part_len, m_ahrs, Parser::ConfigMode::INTERLEAVED_INS);
    len += part_len; // 39 + 19 = 58

    // Append INS (53 bytes)
    part_len = 200 - len;
    fill_simulated_packet(&stream[len], part_len, m_ins, Parser::ConfigMode::INTERLEAVED_INS);
    len += part_len; // 58 + 53 = 111

    Parser::Measurement out;
    int imu_cnt = 0, ahrs_cnt = 0, ins_cnt = 0;

    // Parse the stream
    for (size_t i = 0; i < len; i++) {
        parser.parse_bytes(&stream[i], 1, out);
        
        if (out.type == Parser::MeasurementType::IMU) imu_cnt++;
        else if (out.type == Parser::MeasurementType::AHRS) ahrs_cnt++;
        else if (out.type == Parser::MeasurementType::INS) ins_cnt++;
    }

    // Verify we found exactly one of each, in the correct order context
    EXPECT_EQ(imu_cnt, 1) << "Failed to parse IMU in mixed stream";
    EXPECT_EQ(ahrs_cnt, 1) << "Failed to parse AHRS in mixed stream";
    EXPECT_EQ(ins_cnt, 1) << "Failed to parse INS in mixed stream";
}

// REQ: "Feed a valid 53-byte INS packet 1 byte at a time" [Spec 7.1.B]
TEST(SensAItionParser, Interleaved_INS_Fragmentation)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto in = default_measurement(Parser::MeasurementType::INS);
    
    uint8_t packet[100];
    size_t len = 100;
    fill_simulated_packet(packet, len, in, Parser::ConfigMode::INTERLEAVED_INS);

    EXPECT_EQ(len, 53u); // Verify Spec Size

    Parser::Measurement out;
    
    // Feed bytes 0 to 51 (Should allow no output)
    for (size_t i = 0; i < len - 1; i++) {
        parser.parse_bytes(&packet[i], 1, out);
        EXPECT_EQ(out.type, Parser::MeasurementType::UNINITIALIZED) 
            << "INS Parser triggered prematurely at index " << i;
    }

    // Feed last byte (52) -> Should output INS
    parser.parse_bytes(&packet[len-1], 1, out);
    EXPECT_EQ(out.type, Parser::MeasurementType::INS);
}

// REQ: "Verify int32 1e-7 Lat/Lon extraction match manual specifications" [Spec 7.1.A]
// REQ: "Verify int32 1e-7 Lat/Lon extraction match manual specifications" [Spec 7.1.A]
TEST(SensAItionParser, Interleaved_INS_GoldenCoordinates)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    Parser::Measurement out;

    // Manually construct Packet 2 (INS) - 53 Bytes Total
    // Layout: Header(1) + ID(1) + Payload(50) + CRC(1)
    uint8_t packet[53] = {0};
    
    packet[0] = 0xFA; // Header
    packet[1] = 0x02; // ID = INS

    // Payload starts at packet[2].
    // Lat is at Payload Byte 9. -> Packet Index 2 + 9 = 11.
    // Lon is at Payload Byte 13 -> Packet Index 2 + 13 = 15.
    
    // Target Lat: 59.3293230 -> 593293230 -> 0x235CEFAE
    packet[11] = 0x23; 
    packet[12] = 0x5C; 
    packet[13] = 0xEF; 
    packet[14] = 0xAE;

    // Target Lon: 18.0685810 -> 180685810 -> 0x0AC50BF2
    packet[15] = 0x0A;
    packet[16] = 0xC5;
    packet[17] = 0x0B;
    packet[18] = 0xF2;

    // Recalculate CRC (XOR of ID + Payload, indices 1 to 51)
    uint8_t checksum = 0;
    for(int i=1; i<52; i++) checksum ^= packet[i];
    packet[52] = checksum;

    // Parse
    bool parsed = false;
    for(size_t i=0; i<53; i++) {
        parser.parse_bytes(&packet[i], 1, out);
        if (out.type == Parser::MeasurementType::INS) parsed = true;
    }

    ASSERT_TRUE(parsed);
    
    // Verify 1e-7 scaling integers (Big Endian extraction check)
    EXPECT_EQ(out.location.lat, 593293230) << "INS Latitude Extraction Failed";
    EXPECT_EQ(out.location.lng, 180685810) << "INS Longitude Extraction Failed";
}

// Verifies parser correctly handles negative values (South, West, Deceleration)
TEST(SensAItionParser, Interleaved_Data_StressTest)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto in = default_measurement(Parser::MeasurementType::INS);
    
    // Set Negative Values (South, West, Negative Velocity)
    in.location.lat = -593293230; // South
    in.location.lng = -180685810; // West
    in.velocity_ned.x = -15.5f;   // Moving South fast
    in.velocity_ned.z = -2.5f;    // Climbing (Negative Down)

    uint8_t buffer[100];
    size_t len = 100;
    
    fill_simulated_packet(buffer, len, in, Parser::ConfigMode::INTERLEAVED_INS);

    Parser::Measurement out;
    parser.parse_bytes(buffer, len, out);

    EXPECT_EQ(out.type, Parser::MeasurementType::INS);
    
    // Verify Sign Preservation
    EXPECT_EQ(out.location.lat, -593293230) << "Failed to preserve sign on Latitude";
    EXPECT_EQ(out.location.lng, -180685810) << "Failed to preserve sign on Longitude";
    
    EXPECT_NEAR(out.velocity_ned.x, -15.5f, 0.01f) << "Failed to preserve sign on Velocity X";
    EXPECT_NEAR(out.velocity_ned.z, -2.5f, 0.01f)  << "Failed to preserve sign on Velocity Z";
}

// TEST 2: Noise Recovery Test (Option 2)
// Verifies parser resets cleanly when garbage bytes appear between valid packets
TEST(SensAItionParser, Interleaved_NoiseRecovery)
{
    Parser parser(Parser::ConfigMode::INTERLEAVED_INS);
    auto in = default_measurement(Parser::MeasurementType::INS);
    
    uint8_t valid_packet[100];
    size_t valid_len = 100;
    fill_simulated_packet(valid_packet, valid_len, in, Parser::ConfigMode::INTERLEAVED_INS);

    // Create stream: [Junk Bytes] + [Valid Packet]
    uint8_t stream[200];
    memset(stream, 0xEE, 20); // 20 bytes of junk
    memcpy(&stream[20], valid_packet, valid_len);
    
    Parser::Measurement out;
    bool found = false;

    // Feed junk then valid data
    for (size_t i = 0; i < 20 + valid_len; i++) {
        parser.parse_bytes(&stream[i], 1, out);
        if (out.type == Parser::MeasurementType::INS) {
            found = true;
        }
    }

    EXPECT_TRUE(found) << "Parser failed to recover from initial noise stream";
}

// TEST 3: Partial Legacy Stream (Option 3)
// Verifies backwards compatibility when IMU packet arrives in chunks
TEST(SensAItionParser, Legacy_IMU_PartialStream)
{
    Parser parser(Parser::ConfigMode::IMU);
    auto in = default_measurement(Parser::MeasurementType::IMU);
    
    uint8_t packet[100];
    size_t len = 100;
    fill_simulated_packet(packet, len, in, Parser::ConfigMode::IMU); // Note: ConfigMode::IMU

    Parser::Measurement out;
    size_t chunk_size = 5;
    bool found = false;

    // Feed in small chunks
    for (size_t i = 0; i < len; i += chunk_size) {
        size_t remaining = len - i;
        size_t this_chunk = (remaining < chunk_size) ? remaining : chunk_size;
        
        parser.parse_bytes(&packet[i], this_chunk, out);
        
        if (out.type == Parser::MeasurementType::IMU) {
            found = true;
        }
    }

    EXPECT_TRUE(found) << "Legacy Parser failed to handle fragmented stream";
}

AP_GTEST_MAIN()