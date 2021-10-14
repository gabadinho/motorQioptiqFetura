#include <gtest/gtest.h>

#include "QioptiqFeturaPlusMotorDriver.h"



#define STRING_BUFFER_SIZE 256



TEST(ResponseParse, Sync) {
    const char buffer[] = { 0x0D };
    bool res = FeturaPlusController::gotSynched(buffer, sizeof(buffer));
    ASSERT_EQ(true, res);
}

TEST(ResponseParse, Unsync1) {
    const char buffer[] = { 0x0D, 0x0D };
    bool res = FeturaPlusController::gotSynched(buffer, sizeof(buffer));
    ASSERT_EQ(false, res);
}

TEST(ResponseParse, Unsync2) {
    const char buffer[] = { 0x00 };
    bool res = FeturaPlusController::gotSynched(buffer, sizeof(buffer));
    ASSERT_EQ(false, res);
}



TEST(ResponseParse, Acknowledged) {
    const char buffer[] = { 0x4F };
    bool wrong_reply;
    bool res = FeturaPlusAxis::gotAcknowledged(buffer, sizeof(buffer), wrong_reply);
    ASSERT_EQ(false, wrong_reply);
    ASSERT_EQ(true, res);
}

TEST(ResponseParse, Acknowledge1) {
    const char buffer[] = { 0x4F, 0x4F };
    bool wrong_reply;
    bool res = FeturaPlusAxis::gotAcknowledged(buffer, sizeof(buffer), wrong_reply);
    ASSERT_EQ(true, wrong_reply);
    ASSERT_EQ(false, res);
}

TEST(ResponseParse, Acknowledge2) {
    const char buffer[] = { 0x0D };
    bool wrong_reply;
    bool res = FeturaPlusAxis::gotAcknowledged(buffer, sizeof(buffer), wrong_reply);
    ASSERT_EQ(false, wrong_reply);
    ASSERT_EQ(false, res);
}



TEST(ResponseParse, SerialNumber) {
    const char buffer[] = { 0x4F, 0x0C, 0x00, 0x11, -76, 0x05, 0x00, 0x10, 0x03, -78, 0x04, 0x03, 0x02, 0x01, -91 };
    unsigned serial_number;
    bool wrong_reply, wrong_prefix, wrong_checksum;
    bool res = FeturaPlusController::extractSerialNumber(buffer, sizeof(buffer), serial_number, wrong_reply, wrong_prefix, wrong_checksum);
    ASSERT_EQ(false, wrong_reply);
    ASSERT_EQ(false, wrong_prefix);
    ASSERT_EQ(false, wrong_checksum);
    ASSERT_EQ(true, res);
    ASSERT_EQ(0x04030201, serial_number);
}



TEST(ResponseParse, Firmware) {
    const char buffer[] = { 0x4F, 0x0C, 0x00, 0x11, -76, 0x05, 0x00, 0x10, 0x03, -76, 0x00, 0x02, 0x01, 0x10, -80 };
    unsigned firmware_low, firmware_high;
    bool wrong_reply, wrong_prefix, wrong_checksum;
    bool res = FeturaPlusController::extractFirmware(buffer, sizeof(buffer), firmware_low, firmware_high, wrong_reply, wrong_prefix, wrong_checksum);
    ASSERT_EQ(false, wrong_reply);
    ASSERT_EQ(false, wrong_prefix);
    ASSERT_EQ(false, wrong_checksum);
    ASSERT_EQ(true, res);
    ASSERT_EQ(0x0002, firmware_low);
    ASSERT_EQ(0x0110, firmware_high);
}



TEST(ResponseParse, MoveDone) {
    const char buffer[] =  { 0x4F, 0x08, 0x00, 0x11, -44, 0x01, 0x03, -20, 0x00, 0x01, -34};
    bool got_timeout, wrong_reply, wrong_prefix, wrong_checksum;
    bool res = FeturaPlusAxis::extractMoveTimeout(buffer, sizeof(buffer), got_timeout, wrong_reply, wrong_prefix, wrong_checksum);
    ASSERT_EQ(false, got_timeout);
    ASSERT_EQ(false, wrong_reply);
    ASSERT_EQ(false, wrong_prefix);
    ASSERT_EQ(false, wrong_checksum);
    ASSERT_EQ(true, res);
}

TEST(ResponseParse, MoveTimeout) {
    const char buffer[] =  { 0x4F, 0x08, 0x00, 0x11, -44, 0x01, 0x03, -20, 0x00, 0x00, -35};
    bool got_timeout, wrong_reply, wrong_prefix, wrong_checksum;
    bool res = FeturaPlusAxis::extractMoveTimeout(buffer, sizeof(buffer), got_timeout, wrong_reply, wrong_prefix, wrong_checksum);
    ASSERT_EQ(true, got_timeout);
    ASSERT_EQ(false, wrong_reply);
    ASSERT_EQ(false, wrong_prefix);
    ASSERT_EQ(false, wrong_checksum);
    ASSERT_EQ(true, res);
}



TEST(ResponseParse, Homed) {
    const char buffer[] = { 0x4F, 0x0A, 0x00, 0x11, -76, 0x04, 0x00, 0x10, 0x03, -64, 0x00, 0x01, -89 };
    bool is_homed, wrong_reply, wrong_prefix, wrong_checksum;
    bool res = FeturaPlusAxis::extractHomedState(buffer, sizeof(buffer), is_homed, wrong_reply, wrong_prefix, wrong_checksum);
    ASSERT_EQ(false, wrong_reply);
    ASSERT_EQ(false, wrong_prefix);
    ASSERT_EQ(false, wrong_checksum);
    ASSERT_EQ(true, res);
    ASSERT_EQ(true, is_homed);
}

TEST(ResponseParse, NotHomed) {
    const char buffer[] = { 0x4F, 0x0A, 0x00, 0x11, -76, 0x04, 0x00, 0x10, 0x03, -64, 0x00, 0x00, -90 };
    bool is_homed, wrong_reply, wrong_prefix, wrong_checksum;
    bool res = FeturaPlusAxis::extractHomedState(buffer, sizeof(buffer), is_homed, wrong_reply, wrong_prefix, wrong_checksum);
    ASSERT_EQ(false, wrong_reply);
    ASSERT_EQ(false, wrong_prefix);
    ASSERT_EQ(false, wrong_checksum);
    ASSERT_EQ(true, res);
    ASSERT_EQ(false, is_homed);
}



TEST(ResponseParse, BusyState) {
    const char buffer[] = { 0x4F, 0x0A, 0x00, 0x11, -76, 0x04, 0x00, 0x10, 0x03, -67, 0x00, 0x01, -92 };
    bool is_busy, wrong_reply, wrong_prefix, wrong_checksum;
    bool res = FeturaPlusAxis::extractBusyState(buffer, sizeof(buffer), is_busy, wrong_reply, wrong_prefix, wrong_checksum);
    ASSERT_EQ(false, wrong_reply);
    ASSERT_EQ(false, wrong_prefix);
    ASSERT_EQ(false, wrong_checksum);
    ASSERT_EQ(true, res);
    ASSERT_EQ(true, is_busy);
}

TEST(ResponseParse, IdleState) {
    const char buffer[] = { 0x4F, 0x0A, 0x00, 0x11, -76, 0x04, 0x00, 0x10, 0x03, -67, 0x00, 0x00, -93 };
    bool is_busy, wrong_reply, wrong_prefix, wrong_checksum;
    bool res = FeturaPlusAxis::extractBusyState(buffer, sizeof(buffer), is_busy, wrong_reply, wrong_prefix, wrong_checksum);
    ASSERT_EQ(false, wrong_reply);
    ASSERT_EQ(false, wrong_prefix);
    ASSERT_EQ(false, wrong_checksum);
    ASSERT_EQ(true, res);
    ASSERT_EQ(false, is_busy);
}




TEST(ResponseParse, Readback1) {
    const char buffer[] = { 0x4F, 0x0A, 0x00, 0x11, -76, 0x04, 0x00, 0x10, 0x03, -56, 0x03, 0x00, -79 };
    bool wrong_reply, wrong_prefix, wrong_checksum;
    int readback;
    bool res = FeturaPlusAxis::extractReadback(buffer, sizeof(buffer), readback, wrong_reply, wrong_prefix, wrong_checksum);
    ASSERT_EQ(false, wrong_reply);
    ASSERT_EQ(false, wrong_prefix);
    ASSERT_EQ(false, wrong_checksum);
    ASSERT_EQ(true, res);
    ASSERT_EQ(768, readback);
}

TEST(ResponseParse, Readback2) {
    const char buffer[] = { 0x4F, 0x0A, 0x00, 0x11, -76, 0x04, 0x00, 0x10, 0x03, -56, 0x00, 0x01, -81};
    bool wrong_reply, wrong_prefix, wrong_checksum;
    int readback;
    bool res = FeturaPlusAxis::extractReadback(buffer, sizeof(buffer), readback, wrong_reply, wrong_prefix, wrong_checksum);
    ASSERT_EQ(false, wrong_reply);
    ASSERT_EQ(false, wrong_prefix);
    ASSERT_EQ(false, wrong_checksum);
    ASSERT_EQ(true, res);
    ASSERT_EQ(1, readback);
}



TEST(CommandBuild, Move1) {
    char buffer[STRING_BUFFER_SIZE];
    size_t nwrite;
    bool res = FeturaPlusAxis::buildMoveCommand(buffer, 210, nwrite);
    unsigned char cmd[] = { 0x06, 0x00, 0x10, 0x21, 0xC7, 0x00, 0xD2, 0xD0 };
    ASSERT_EQ(true, res);
    ASSERT_EQ(sizeof(cmd), nwrite);
    ASSERT_EQ(0, memcmp(cmd, buffer, sizeof(cmd)));
}

TEST(CommandBuild, Move2) {
    char buffer[STRING_BUFFER_SIZE];
    size_t nwrite;
    bool res = FeturaPlusAxis::buildMoveCommand(buffer, 900, nwrite);
    unsigned char cmd[] = { 0x06, 0x00, 0x10, 0x21, 0xC7, 0x03, 0x84, 0x85 };
    ASSERT_EQ(true, res);
    ASSERT_EQ(sizeof(cmd), nwrite);
    ASSERT_EQ(0, memcmp(cmd, buffer, sizeof(cmd)));
}



TEST(Checksum, CalculateSimple) {
    const char cmd[] = { 0x01, 0x00, 0x02, 0x10 };
    unsigned char checksum = FeturaPlusController::calculateChecksum(cmd, sizeof(cmd));
    ASSERT_EQ(0x13, checksum);
}

TEST(Checksum, CalculateOverflow) {
    const char cmd[] = { 0x70, 0x70, 0x71 };
    unsigned char checksum = FeturaPlusController::calculateChecksum(cmd, sizeof(cmd));
    ASSERT_EQ(0x51, checksum);
}



TEST(Checksum, CheckCorrectSimple) {
    const char frame[] = { 0x01, 0x02, 0x02 };
    bool res = FeturaPlusController::checkChecksumAtEnd(frame, sizeof(frame));
    ASSERT_EQ(true, res);
}

TEST(Checksum, CheckCorrectOverflowZero) {
    const char frame[] = { 0x01, -1, 0x01, 0x00 };
    bool res = FeturaPlusController::checkChecksumAtEnd(frame, sizeof(frame));
    ASSERT_EQ(true, res);
}

TEST(Checksum, CheckIncorrectDueFirstByte) {
    const char frame[] = { 0x01, 0x02, 0x03 };
    bool res = FeturaPlusController::checkChecksumAtEnd(frame, sizeof(frame));
    ASSERT_EQ(false, res);
}

