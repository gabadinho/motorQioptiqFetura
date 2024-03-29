/*
FILENAME...   QioptiqFeturaPlusMotorDriver.h
USAGE...      Motor driver support (model 3, asyn) for the Qioptiq Fetura+ optics

Jose G.C. Gabadinho
October 2020
*/

#ifndef _QIOPTIQFETURAPLUSMOTORDRIVER_H_
#define _QIOPTIQFETURAPLUSMOTORDRIVER_H_

#include <stddef.h>

#include <asynMotorController.h>
#include <asynMotorAxis.h>



const unsigned char ACKNOWLEDGE[] = { 0x4F };

const unsigned char CHECK_STATUS_CMD[]    = { 0x08, 0x00, 0x10, 0xB0, 0x04, 0x00, 0x11, 0x03, 0xBD, 0x9D };
const unsigned char STATUS_REPLY_PREFIX[] = { 0x4F, 0x0A, 0x00, 0x11, 0xB4, 0x04, 0x00, 0x10, 0x03, 0xBD };

const unsigned char MOVETO_CMD_PREFIX[]   = { 0x06, 0x00, 0x10, 0x21, 0xC7 };
const unsigned char MOVETO_REPLY_PREFIX[] = { 0x4F, 0x08, 0x00, 0x11, 0xD4, 0x01, 0x03, 0xEC };

const unsigned char READBACK_CMD[]          = { 0x08, 0x00, 0x10, 0xB0, 0x04, 0x00, 0x11, 0x03, 0xC8, 0xA8 };
const unsigned char READBACK_REPLY_PREFIX[] = { 0x4F, 0x0A, 0x00, 0x11, 0xB4, 0x04, 0x00, 0x10, 0x03, 0xC8 };

const unsigned char RESET_CMD[] = { 0x04, 0x10, 0x00, 0x04, 0x02, 0x1A };

const unsigned char CHECKHOME_CMD[]          = { 0x08, 0x00, 0x10, 0xB0, 0x04, 0x00, 0x11, 0x03, 0xC0, 0xA0 };
const unsigned char CHECKHOME_REPLY_PREFIX[] = { 0x4F, 0x0A, 0x00, 0x11, 0xB4, 0x04, 0x00, 0x10, 0x03, 0xC0 };

const unsigned char SERIALNUMBER_CMD[]          = { 0x08, 0x00, 0x10, 0xB0, 0x05, 0x00, 0x11, 0x03, 0xB2, 0x93 };
const unsigned char SERIALNUMBER_REPLY_PREFIX[] = { 0x4F, 0x0C, 0x00, 0x11, 0xB4, 0x05, 0x00, 0x10, 0x03, 0xB2 };

const unsigned char FIRMWARE_CMD[]          = { 0x08, 0x00, 0x10, 0xB0, 0x05, 0x00, 0x11, 0x03, 0xB4, 0x95 };
const unsigned char FIRMWARE_REPLY_PREFIX[] = { 0x4F, 0x0C, 0x00, 0x11, 0xB4, 0x05, 0x00, 0x10, 0x03, 0xB4 };

const unsigned char SYNC_CMD[]   = { 0xFF };
const unsigned char SYNC_REPLY[] = { 0x0D };

const unsigned MOVEMODE_THRESHOLD = 1000;



class FeturaPlusAxis: public asynMotorAxis {

public:
    FeturaPlusAxis(class FeturaPlusController *pC, int axis);

    // These are the methods we override from the base class
    void report(FILE *fp, int level);

    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);

    asynStatus poll(bool *moving);

    // Class-wide methods
    static bool buildMoveCommand(char *buffer, unsigned pos, size_t& cmd_len);

    static bool gotAcknowledged(const char *buffer, size_t nread, bool& wrong_reply);
    static bool extractMoveTimeout(const char *buffer, size_t nread, bool& got_timeout, bool& wrong_reply, bool& wrong_prefix, bool& wrong_checksum);
    static bool extractReadback(const char *buffer, size_t nread, int& readback, bool& wrong_reply, bool& wrong_prefix, bool& wrong_checksum);
    static bool extractHomedState(const char *buffer, size_t nread, bool& is_homed, bool& wrong_reply, bool& wrong_prefix, bool& wrong_checksum);
    static bool extractBusyState(const char *buffer, size_t nread, bool& is_busy, bool& wrong_reply, bool& wrong_prefix, bool& wrong_checksum);

protected:
    // Specific class methods
    void setStatusProblem(asynStatus status);

    int getHomedState();
    int getBusyState();
    int getCurrentPosition();

private:
    FeturaPlusController *pC_; // Pointer to the asynMotorController to which this axis belongs
    
friend class FeturaPlusController;
};



class FeturaPlusController: public asynMotorController {

public:
    FeturaPlusController(const char *portName, const char *asynPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

    // These are the methods we override from the base class
    void report(FILE *fp, int level);

    FeturaPlusAxis* getAxis(asynUser *pasynUser);
    FeturaPlusAxis* getAxis(int axisNo);

    // Class-wide methods
    static bool buildSimpleCommand(char *buffer, const unsigned char cmd[], size_t cmd_len);

    static bool gotSynched(const char *buffer, size_t nread);
    static bool extractSerialNumber(const char *buffer, size_t nread, unsigned& serial_nr, bool& wrong_reply, bool& wrong_prefix, bool& wrong_checksum);
    static bool extractFirmware(const char *buffer, size_t nread, unsigned& firmware_low, unsigned& firmware_high, bool& wrong_reply, bool& wrong_prefix, bool& wrong_checksum);

    static unsigned char calculateChecksum(const char *frame, size_t frame_length);
    static bool checkChecksumAtEnd(const char *frame, size_t frame_length);

protected:
    // Specific class methods
    asynStatus writeReadController(size_t nwrite, size_t *nread);

#define NUM_FETURA_PARAMS 0

private:

friend class FeturaPlusAxis;
};

#endif // _QIOPTIQFETURAPLUSMOTORDRIVER_H_
