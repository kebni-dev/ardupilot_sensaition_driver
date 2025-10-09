#include <AP_gtest.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS_SensAItion_Parser.h>

TEST(SensAItionParser, ValidIMUPacket)
{
    AP_ExternalAHRS_SensAItion_Parser parser;

    // Valid IMU packet: Header + 36 data bytes + checksum
    uint8_t valid_packet[38];
    valid_packet[0] = 0xFA;  // Header

    // Fill with test data (9 sensors × 4 bytes)
    for (int i = 1; i <= 36; i++) {
        valid_packet[i] = i;
    }

    // Calculate XOR checksum
    uint8_t checksum = 0;
    for (int i = 1; i <= 36; i++) {
        checksum ^= valid_packet[i];
    }
    valid_packet[37] = checksum;

    const uint8_t* packet_out;
    size_t packet_size;

    bool result = parser.parse_bytes(valid_packet, sizeof(valid_packet), packet_out, packet_size);

    EXPECT_TRUE(result);
    EXPECT_EQ(packet_size, 36U);
    EXPECT_EQ(parser.get_valid_packets(), 1U);
    EXPECT_EQ(parser.get_parse_errors(), 0U);
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

AP_GTEST_MAIN()
