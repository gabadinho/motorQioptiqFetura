/*
FILENAME...   QioptiqFeturaPlusMotorDriver.cpp
USAGE...      Motor driver support (model 3, asyn) for the Qioptiq Fetura+ optics

Jose G.C. Gabadinho
October 2020
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "QioptiqFeturaPlusMotorDriver.h"

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>



#define RESET_SLEEP        0.501
#define HOMING_SLEEP       1.0
#define CONTROLLER_TIMEOUT 0.11



static const char *driverName = "FeturaPlusOptics";

/** Creates a new FeturaPlusController object.
  *
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPortName      The name of the drvAsynIPPPort that was created previously to connect to the Fetura+ controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
FeturaPlusController::FeturaPlusController(const char *portName, const char *asynPortName, int numAxes, double movingPollPeriod, double idlePollPeriod)
    :asynMotorController(portName, 1, NUM_FETURA_PARAMS, 
                         0,
                         0,
                         ASYN_CANBLOCK,
                         1, /* autoconnect */
                         0, 0) /* Default priority and stack size */ {
    int axis;
    asynStatus status;
    static const char *functionName = "FeturaPlusController";
    size_t nread;

    numAxes = 1; // Force single-axis regardless of what user says

    // Connect to Fetura+ controller
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: Creating Fetura+ controller %s to asyn %s with %d axes\n", driverName, functionName, portName, asynPortName, numAxes);
    status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: cannot connect to Fetura+ controller at asyn %s\n", driverName, functionName, asynPortName);
    } else {
        pasynOctetSyncIO->setInputEos(pasynUserController_, "", 0);
        pasynOctetSyncIO->setOutputEos(pasynUserController_, "", 0);
        
        buildSimpleCommand(this->outString_, SYNC_CMD, sizeof(SYNC_CMD));
        writeReadController(sizeof(SYNC_CMD), &nread);
        if (!gotSynched(this->inString_, nread)) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: unable to synchronize with Fetura+ %s\n", driverName, functionName, portName);
        }
    }

    // Create the axis objects
    for (axis=0; axis<numAxes; axis++) {
        new FeturaPlusAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Reports on status of the driver.
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  *
  * \param[in] fp     The file pointer on which report information will be written
  * \param[in] level  The level of report detail desired
  */
void FeturaPlusController::report(FILE *fp, int level) {
    size_t nread;
    unsigned serialnr = 0, firmlo = 0, firmhi = 0;
    bool reply_check, prefix_check, checksum_check;

    fprintf(fp, "Qioptiq Fetura+ optics controller %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

    if (level > 0) {
        // Retrieve serial number
        buildSimpleCommand(this->outString_, SERIALNUMBER_CMD, sizeof(SERIALNUMBER_CMD));
        writeReadController(sizeof(SERIALNUMBER_CMD), &nread);
        if (extractSerialNumber(this->inString_, nread, serialnr, reply_check, prefix_check, checksum_check)) {
            if (serialnr) {
                fprintf(fp, "  serial number=%X\n", serialnr);
            } else {
                fprintf(fp, "  unknown serial number\n");
            }
        } else {
            if (checksum_check) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong checksum while getting Fetura+ %s serial number\n", this->portName);
            } else if (prefix_check) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while getting Fetura+ %s serial number\n", this->portName);
            } else {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while getting Fetura+ %s serial number (read %zu bytes)\n", this->portName, nread);
            }
        }

        // Retrieve firmware version
        buildSimpleCommand(this->outString_, FIRMWARE_CMD, sizeof(FIRMWARE_CMD));
        writeReadController(sizeof(FIRMWARE_CMD), &nread);
        if (extractFirmware(this->inString_, nread, firmlo, firmhi, reply_check, prefix_check, checksum_check)) {
            if ((firmhi) || (firmlo)) {
                fprintf(fp, "  firmware version=%d.%d\n", firmhi, firmlo);
            } else {
                fprintf(fp, "  unknown firmware version\n");
            }
        } else {
            if (checksum_check) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong checksum while getting Fetura+ %s firmware version\n", this->portName);
            } else if (prefix_check) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while getting Fetura+ %s firmware version\n", this->portName);
            } else {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while getting Fetura+ %s firmware version (read %zu bytes)\n", this->portName, nread);
            }
        }
    }

    // Call the base class method
    asynMotorController::report(fp, level);
}

/** Returns a pointer to an FeturaPlusAxis object.
  *
  * \param[in] pasynUser  An asynUser structure that encodes the axis index number.
  *
  * \return A FeturaPlusAxis object or NULL if the axis number encoded in pasynUser is invalid
  */
FeturaPlusAxis* FeturaPlusController::getAxis(asynUser *pasynUser) {
    return static_cast<FeturaPlusAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an FeturaPlusAxis object.
  *
  * \param[in] axisNo  The axis index number.
  *
  * \return A FeturaPlusAxis object or NULL if the axis number encoded in pasynUser is invalid
  */
FeturaPlusAxis* FeturaPlusController::getAxis(int axisNo) {
    return static_cast<FeturaPlusAxis*>(asynMotorController::getAxis(axisNo));
}

/** All the following methods generate a command string to be sent to the controller.
  *
  */
bool FeturaPlusController::buildSimpleCommand(char *buffer, const unsigned char cmd[], size_t cmd_len) {
    if ((!buffer) || (!cmd) || (cmd_len<=1)) {
        return false;
    }
    memcpy(buffer, cmd, cmd_len);
    return true;
}

/** Verifies the synchronize reply.
  *
  * \param[in] frame  Array of bytes
  * \param[in] nread  Size/length of array
  *
  * \return true if data frame matches the expected reply to a synchronize command
  */
bool FeturaPlusController::gotSynched(const char *buffer, size_t nread) {
    bool res = false;
    if (nread == sizeof(SYNC_REPLY)) {
        if (!memcmp(buffer, SYNC_REPLY, sizeof(SYNC_REPLY))) {
            res = true;
        }
    }
    return res;
}

/** Extracts the controller serial number from a data frame.
  *
  * \param[in] frame  Array of bytes
  * \param[in] nread  Size/length of array
  *
  * \param[out] serial_nr       Extracted serial number from the frame
  * \param[out] wrong_reply     Set to true if frame is not of correct size
  * \param[out] wrong_prefix    Set to true if frame prefix doesn't match the expected for the serial number reply
  * \param[out] wrong_checksum  Set to true if frame contained a miscalculated checksum
  *
  * \return true if frame prefix is correct and calculated checksum matches the one embedded in the frame
  */
bool FeturaPlusController::extractSerialNumber(const char *buffer, size_t nread, unsigned& serial_nr, bool& wrong_reply, bool& wrong_prefix, bool& wrong_checksum) {
    bool res = false;
    unsigned sernr = 0;

    wrong_reply    = false;
    wrong_prefix   = false;
    wrong_checksum = false;
    if (!buffer) {
        return false;
    }

    if (nread == sizeof(SERIALNUMBER_REPLY_PREFIX)+5) {
        if (!memcmp(buffer, SERIALNUMBER_REPLY_PREFIX, sizeof(SERIALNUMBER_REPLY_PREFIX))) {
            if (checkChecksumAtEnd(buffer, nread)) {
                sernr = ((buffer[sizeof(SERIALNUMBER_REPLY_PREFIX)]) << 24) + 
                        ((buffer[sizeof(SERIALNUMBER_REPLY_PREFIX)+1]) << 16) + 
                        ((buffer[sizeof(SERIALNUMBER_REPLY_PREFIX)+2]) << 8) + 
                        (buffer[sizeof(SERIALNUMBER_REPLY_PREFIX)+3]);
                res = true;
            } else {
                wrong_checksum = true;
            }
        } else {
            wrong_prefix = true;
        }
    } else {
        wrong_reply = true;
    }

    serial_nr = sernr;
    return res;
}

/** Extracts the firmware version from a data frame.
  *
  * \param[in] frame  Array of bytes
  * \param[in] nread  Size/length of array
  *
  * \param[out] firmware_low    Extracted firmware version least-significant number
  * \param[out] firmware_high   Extracted firmware version most-significant number
  * \param[out] wrong_reply     Set to true if frame is not of correct size
  * \param[out] wrong_prefix    Set to true if frame prefix doesn't match the expected for the firmware reply
  * \param[out] wrong_checksum  Set to true if frame contained a miscalculated checksum
  *
  * \return true if frame prefix is correct and calculated checksum matches the one embedded in the frame
  */
bool FeturaPlusController::extractFirmware(const char *buffer, size_t nread, unsigned& firmware_low, unsigned& firmware_high, bool& wrong_reply, bool& wrong_prefix, bool& wrong_checksum) {
    bool res = false;
    unsigned firlo = 0, firhi = 0;

    wrong_reply    = false;
    wrong_prefix   = false;
    wrong_checksum = false;
    if (!buffer) {
        return false;
    }

    if (nread == sizeof(FIRMWARE_REPLY_PREFIX)+5) {
        if (!memcmp(buffer, FIRMWARE_REPLY_PREFIX, sizeof(FIRMWARE_REPLY_PREFIX))) {
            if (checkChecksumAtEnd(buffer, nread)) {
                firhi = ((buffer[sizeof(FIRMWARE_REPLY_PREFIX)+2]) << 8) + 
                        (buffer[sizeof(FIRMWARE_REPLY_PREFIX)+3]);
                firlo = ((buffer[sizeof(FIRMWARE_REPLY_PREFIX)]) << 8) + 
                        (buffer[sizeof(FIRMWARE_REPLY_PREFIX)+1]);
                res = true;
            } else {
                wrong_checksum = true;
            }
        } else {
            wrong_prefix = true;
        }
    } else {
        wrong_reply = true;
    }

    firmware_low  = firlo;
    firmware_high = firhi;
    return res;
}

/** Calculates the checksum of a data frame.
  *
  * \param[in] frame         Array of bytes
  * \param[in] frame_length  Size/length of array
  *
  * \return Checksum: least-significative byte of the sum of all bytes
  */
unsigned char FeturaPlusController::calculateChecksum(const char *frame, size_t frame_length) {
    unsigned int sum = 0;

    for (size_t i=0; i<frame_length; i++) {
        sum += (unsigned char)(frame[i]);
    }
    sum = sum & 0xFF;
    return sum;
}

/** Verifies the checksum at the end of a data frame.
  *
  * \param[in] frame         Array of bytes; checksum is expected as the very last byte
  *                          the first byte (acknowledgement 0x4F is discarded)
  * \param[in] frame_length  Size/length of array, including acknowledgement and checksum bytes
  *
  * \return true if calculated checksum matches the one embedded in the data frame
  */
bool FeturaPlusController::checkChecksumAtEnd(const char *frame, size_t frame_length) {
    unsigned char calculated_checksum = calculateChecksum(frame+1, frame_length-2);
    unsigned char given_checksum = (unsigned char)(frame[frame_length-1]);
    return (calculated_checksum==given_checksum);
}

/** Wrapper of writeReadController with explicit output length, avoiding usage of strlen().
  *
  * \param[in] nwrite  Number of bytes to write
  *
  * \param[out] nread  Number of bytes read
  *
  * \return Result of pasynOctetSyncIO::writeRead
  */
asynStatus FeturaPlusController::writeReadController(size_t nwrite, size_t *nread) {
    size_t nwrote;
    int eomReason;

    *nread = 0;
    return pasynOctetSyncIO->writeRead(pasynUserController_,
                                       outString_, nwrite,
                                       inString_, sizeof(inString_),
                                       CONTROLLER_TIMEOUT,
                                       &nwrote, nread, &eomReason);
}



// These are the FeturaPlusAxis methods

/** Creates a new FeturaPlusAxis object.
  *
  * \param[in] pC      Pointer to the FeturaPlusController to which this axis belongs
  * \param[in] axisNo  Index number of this axis, range 0 to pC->numAxes_-1
  */
FeturaPlusAxis::FeturaPlusAxis(FeturaPlusController *pC, int axisNo): asynMotorAxis(pC, axisNo), pC_(pC) {
    int home_done = -1;

    home_done = getHomedState();
    if (home_done != -1) {
        this->setIntegerParam(pC_->motorStatusHomed_, home_done);
    }

    callParamCallbacks();
}

/** Reports on status of the driver.
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorAxis::report()
  *
  * \param[in] fp     The file pointer on which report information will be written
  * \param[in] level  The level of report detail desired
  */
void FeturaPlusAxis::report(FILE *fp, int level) {
    int home_done = -1;

    if (level > 0) {
        home_done = getHomedState();
        if (home_done!=-1) {
            this->setIntegerParam(pC_->motorStatusHomed_, home_done);
            if (home_done) {
                fprintf(fp, "  Homing done\n");
            } else {
                fprintf(fp, "  Homing in progress\n");
            } 
        } else {
            fprintf(fp, "  Homing unknown\n");
        }
    }

    asynMotorAxis::report(fp, level);
}

/** Moves to a different zoom position.
  *
  * \param[in] position      The desired target position
  * \param[in] relative      1 for relative position
  * \param[in] minVelocity   Motion parameter
  * \param[in] maxVelocity   Motion parameter
  * \param[in] acceleration  Motion parameter
  *
  * \return The result of callParamCallbacks()
  */
asynStatus FeturaPlusAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration) {
    asynStatus status = asynError;
    size_t nread, nwrite;
    int moveto_pos = position;
    bool timeout, reply_check, prefix_check, checksum_check;

    // First check if ready
    if (getBusyState() == 0) {
        setIntegerParam(pC_->motorStatusDone_, 0);

        // Select how to move: CAM: default FZM: Fast Zoom Mode or Continuous Zoom Mode
        if (maxVelocity > MOVEMODE_THRESHOLD) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Moving Fetura+ %s to %f in CZM\n", pC_->portName, position);
            position += MOVEMODE_THRESHOLD;
        } else {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Moving Fetura+ %s to %f in FZM\n", pC_->portName, position);
        }

        buildMoveCommand(pC_->outString_, moveto_pos, nwrite);
        pC_->writeReadController(nwrite, &nread);
        if (extractMoveTimeout(pC_->inString_, nread, timeout, reply_check, prefix_check, checksum_check)) {
            if (timeout) {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Timeout while moving Fetura+ %s\n", pC_->portName);
            } else {
                status = asynSuccess;
            }
        } else {
            if (checksum_check) {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong checksum while moving Fetura+ %s\n", pC_->portName);
            } else if (prefix_check) {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while moving Fetura+ %s\n", pC_->portName);
            } else {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while moving Fetura+ %s (read %zu bytes)\n", pC_->portName, nread);
            }
        }
    }

    setStatusProblem(status);

    return callParamCallbacks();
}

/** Homes the motor by re-initializing the controller.
  *
  * \param[in] minVelocity   Motion parameter
  * \param[in] maxVelocity   Motion parameter
  * \param[in] acceleration  Motion parameter
  * \param[in] forwards      1 if user wants to home forward, 0 for reverse
  *
  * \return The result of callParamCallbacks()
  */
asynStatus FeturaPlusAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    asynStatus status = asynError;
    size_t nread;
    bool reply_check;
    int home_done;

    // First check if ready
    if (getBusyState() == 0) {
        setIntegerParam(pC_->motorStatusDone_, 0);
        setDoubleParam(pC_->motorPosition_, 0);

        // Reset controller
        pC_->buildSimpleCommand(pC_->outString_, RESET_CMD, sizeof(RESET_CMD));
        pC_->writeReadController(sizeof(RESET_CMD), &nread);
        if (gotAcknowledged(pC_->inString_, nread, reply_check)) {
            // Wait and flush communications
            epicsThreadSleep(RESET_SLEEP);
            pasynOctetSyncIO->flush(pC_->pasynUserController_);

            epicsThreadSleep(HOMING_SLEEP);
            home_done = getHomedState();
            if (home_done != -1) {
                this->setIntegerParam(pC_->motorStatusHomed_, home_done);
            }
        } else {
            if (reply_check) {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while homing Fetura+ %s (read %zu bytes)\n", pC_->portName, nread);
            } else {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Unrecognized reply while homing Fetura+ %s\n", pC_->portName);
            }
        }
    }

    setStatusProblem(status);

    return callParamCallbacks();
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  *
  * \param[out] moving  A flag that is set indicating that the axis is moving (1) or done (0).
  *
  * \return The result of callParamCallbacks()
  */
asynStatus FeturaPlusAxis::poll(bool *moving) { 
    asynStatus status = asynError;
    int status_done, errors=0, readback, is_busy;

    /* Check status: ready or busy  */
    is_busy = getBusyState();
    if (is_busy != -1) {
        *moving = is_busy;
        pC_->getIntegerParam(axisNo_, pC_->motorStatusDone_, &status_done);
        if (status_done != (1-is_busy)) {
            this->setIntegerParam(pC_->motorStatusDone_, 1-is_busy);
        }
    } else {
        errors++;
    }

    /* Retrieve readback  */
    readback = getCurrentPosition();
    if (readback != -1) {
        setDoubleParam(pC_->motorPosition_, readback);
    } else {
        errors++;
    }

    if (!errors) {
        status = asynSuccess;
    }
    setStatusProblem(status);

    return callParamCallbacks();
}

/** All the following methods generate a command string to be sent to the controller.
  *
  */
bool FeturaPlusAxis::buildMoveCommand(char *buffer, unsigned pos, size_t& cmd_len) {
    unsigned char checksum;

    if (!buffer) {
        return false;
    }

    memcpy(buffer, MOVETO_CMD_PREFIX, sizeof(MOVETO_CMD_PREFIX));
    buffer[sizeof(MOVETO_CMD_PREFIX)] = (pos & 0xFF00)>>8;
    buffer[sizeof(MOVETO_CMD_PREFIX)+1] = (pos & 0xFF);
    checksum = FeturaPlusController::calculateChecksum(buffer, sizeof(MOVETO_CMD_PREFIX)+2);
    buffer[sizeof(MOVETO_CMD_PREFIX)+2] = checksum;

    cmd_len = sizeof(MOVETO_CMD_PREFIX)+3;

    return true;
}

/** Verifies an acknowledgement reply.
  *
  * \param[in] frame  Array of bytes
  * \param[in] nread  Size/length of array
  *
  * \param[out] wrong_reply  Set to true if frame is not of correct size
  *
  * \return true if data frame matches the expected reply to a home (or move) command
  */
bool FeturaPlusAxis::gotAcknowledged(const char *buffer, size_t nread, bool& wrong_reply) {
    bool res = false;

    wrong_reply = false;
    if (!buffer) {
        return false;
    }

    if (nread == sizeof(ACKNOWLEDGE)) {
        if (!memcmp(buffer, ACKNOWLEDGE, sizeof(ACKNOWLEDGE))) {
            res = true;
        }
    } else {
        wrong_reply = true;
    }

    return res;
}

/** Extracts the readback position from a data frame.
  *
  * \param[in] frame  Array of bytes
  * \param[in] nread  Size/length of array
  *
  * \param[out] readback        Extracted current zoom position
  * \param[out] wrong_reply     Set to true if frame is not of correct size
  * \param[out] wrong_prefix    Set to true if frame prefix doesn't match the expected for the readback reply
  * \param[out] wrong_checksum  Set to true if frame contained a miscalculated checksum
  *
  * \return true if frame prefix is correct and calculated checksum matches the one embedded in the frame
  */
bool FeturaPlusAxis::extractReadback(const char *buffer, size_t nread, int& readback, bool& wrong_reply, bool& wrong_prefix, bool& wrong_checksum) {
    bool res = false;
    int rb = -1;

    wrong_reply    = false;
    wrong_prefix   = false;
    wrong_checksum = false;
    if (!buffer) {
        return false;
    }

    if (nread == sizeof(READBACK_REPLY_PREFIX)+3) {
        if (!memcmp(buffer, READBACK_REPLY_PREFIX, sizeof(READBACK_REPLY_PREFIX))) {
            if (FeturaPlusController::checkChecksumAtEnd(buffer, nread)) {
                rb = buffer[sizeof(READBACK_REPLY_PREFIX)] << 8;
                rb += (unsigned char)(buffer[sizeof(READBACK_REPLY_PREFIX)+1]);
                res = true;
            } else {
                wrong_checksum = true;
            }
        } else {
            wrong_prefix = true;
        }
    } else {
        wrong_reply = true;
    }

    readback = rb;
    return res;
}

/** Extracts the homed status from a data frame.
  *
  * \param[in] frame  Array of bytes
  * \param[in] nread  Size/length of array
  *
  * \param[out] is_homed        Extracted homed status
  * \param[out] wrong_reply     Set to true if frame is not of correct size
  * \param[out] wrong_prefix    Set to true if frame prefix doesn't match the expected for the homed reply
  * \param[out] wrong_checksum  Set to true if frame contained a miscalculated checksum
  *
  * \return true if frame prefix is correct and calculated checksum matches the one embedded in the frame
  */
bool FeturaPlusAxis::extractHomedState(const char *buffer, size_t nread, bool& is_homed, bool& wrong_reply, bool& wrong_prefix, bool& wrong_checksum) {
    bool res = false;
    int hom = -1;

    wrong_reply    = false;
    wrong_prefix   = false;
    wrong_checksum = false;
    if (!buffer) {
        return false;
    }

    if (nread == sizeof(CHECKHOME_REPLY_PREFIX)+3) {
        if (!memcmp(buffer, CHECKHOME_REPLY_PREFIX, sizeof(CHECKHOME_REPLY_PREFIX))) {
            if ((buffer[sizeof(CHECKHOME_REPLY_PREFIX)] == 0x00) && 
                ((buffer[sizeof(CHECKHOME_REPLY_PREFIX)+1] == 0x00) || (buffer[sizeof(CHECKHOME_REPLY_PREFIX)+1] == 0x01))) {
                if (FeturaPlusController::checkChecksumAtEnd(buffer, nread)) {
                    hom = buffer[sizeof(CHECKHOME_REPLY_PREFIX)+1];
                    res = true;
                } else {
                    wrong_checksum = true;
                }
            } else {
                wrong_prefix = true;
            }
        } else {
            wrong_prefix = true;
        }
    } else {
        wrong_reply = true;
    }

    is_homed = hom;
    return res;
}

/** Extracts the controller busy status from a data frame.
  *
  * \param[in] frame  Array of bytes
  * \param[in] nread  Size/length of array
  *
  * \param[out] is_busy         Extracted busy status
  * \param[out] wrong_reply     Set to true if frame is not of correct size
  * \param[out] wrong_prefix    Set to true if frame prefix doesn't match the expected for the busy reply
  * \param[out] wrong_checksum  Set to true if frame contained a miscalculated checksum
  *
  * \return true if frame prefix is correct and calculated checksum matches the one embedded in the frame
  */
bool FeturaPlusAxis::extractBusyState(const char *buffer, size_t nread, bool& is_busy, bool& wrong_reply, bool& wrong_prefix, bool& wrong_checksum) {
    bool res = false;
    int busy = -1;

    wrong_reply    = false;
    wrong_prefix   = false;
    wrong_checksum = false;
    if (!buffer) {
        return false;
    }

    if (nread == sizeof(STATUS_REPLY_PREFIX)+3) {
        if (!memcmp(buffer, STATUS_REPLY_PREFIX, sizeof(STATUS_REPLY_PREFIX))) {
            if ((buffer[sizeof(STATUS_REPLY_PREFIX)] == 0x00) && 
                ((buffer[sizeof(STATUS_REPLY_PREFIX)+1] == 0x00) || (buffer[sizeof(STATUS_REPLY_PREFIX)+1] == 0x01))) {
                if (FeturaPlusController::checkChecksumAtEnd(buffer, nread)) {
                    busy = buffer[sizeof(STATUS_REPLY_PREFIX)+1];
                    res = true;
                } else {
                    wrong_checksum = true;
                }
            } else {
                wrong_prefix = true;
            }
        } else {
            wrong_prefix = true;
        }
    } else {
        wrong_reply = true;
    }

    is_busy = busy;
    return res;
}

/** Extracts the controller move-request status from a data frame.
  *
  * \param[in] frame  Array of bytes
  * \param[in] nread  Size/length of array
  *
  * \param[out] got_timeou      Extracted move-request/got-timeout status
  * \param[out] wrong_reply     Set to true if frame is not of correct size
  * \param[out] wrong_prefix    Set to true if frame prefix doesn't match the expected for the move reply
  * \param[out] wrong_checksum  Set to true if frame contained a miscalculated checksum
  *
  * \return true if frame prefix is correct and calculated checksum matches the one embedded in the frame
  */
bool FeturaPlusAxis::extractMoveTimeout(const char *buffer, size_t nread, bool& got_timeout, bool& wrong_reply, bool& wrong_prefix, bool& wrong_checksum) {
    bool res = false;
    int timeout = 1;

    wrong_reply    = false;
    wrong_prefix   = false;
    wrong_checksum = false;
    if (!buffer) {
        return false;
    }

    if (nread == sizeof(ACKNOWLEDGE)) {
        if (memcmp(buffer, ACKNOWLEDGE, sizeof(ACKNOWLEDGE))) {
            wrong_prefix = true;
        } else {
            timeout = 0;
            res = true;
        }
    } else if (nread == sizeof(MOVETO_REPLY_PREFIX)+3) {
        if ((!memcmp(buffer, MOVETO_REPLY_PREFIX, sizeof(MOVETO_REPLY_PREFIX))) && (buffer[sizeof(MOVETO_REPLY_PREFIX)] == 0x00) &&
                    ((buffer[sizeof(MOVETO_REPLY_PREFIX)+1] == 0x00) || (buffer[sizeof(MOVETO_REPLY_PREFIX)+1] == 0x01))) {
            if (FeturaPlusController::checkChecksumAtEnd(buffer, nread)) {
                timeout = 1-buffer[sizeof(MOVETO_REPLY_PREFIX)+1];
                res = true;
            } else {
                wrong_checksum = true;
            }
        } else {
            wrong_prefix = true;
        }
    } else {
        wrong_reply = true;
    }

    got_timeout = timeout;
    return res;
}

/** Raises the motor record problem status.
  *
  * \param[in] status  Last operation status
  */
void FeturaPlusAxis::setStatusProblem(asynStatus status) {
    int status_problem;

    pC_->getIntegerParam(axisNo_, pC_->motorStatus_, &status_problem);
    if ((status != asynSuccess) && (!status_problem)) {
        this->setIntegerParam(pC_->motorStatusProblem_, 1);
    } else if ((status == asynSuccess) && (status_problem)) {
        this->setIntegerParam(pC_->motorStatusProblem_, 0);
    }
}

/** Retrieves the zoom motor homed status.
  *
  * \return 1 for controller homed, 0 for homing in process, -1 for unknown homed status
  */
int FeturaPlusAxis::getHomedState() {
    int home_done=-1;
    size_t nread;
    bool homed, reply_check, prefix_check, checksum_check;

    // Retrieve homing vs. homed
    pC_->buildSimpleCommand(pC_->outString_, CHECKHOME_CMD, sizeof(CHECKHOME_CMD));
    pC_->writeReadController(sizeof(CHECKHOME_CMD), &nread);
    if (!extractHomedState(pC_->inString_, nread, homed, reply_check, prefix_check, checksum_check)) {
        if (checksum_check) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong checksum while retrieving Fetura+ %s homed state\n", pC_->portName);
        } else if (prefix_check) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while retrieving Fetura+ %s homed state\n", pC_->portName);
        } else {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while retrieving Fetura+ %s homed state (read %zu bytes)\n", pC_->portName, nread);
        }
    } else {
        home_done = homed;
    }

    return home_done;
}

/** Retrieves the zoom motor ready vs. busy status.
  *
  * \return 1 for controller busy, 0 for controller ready, -1 for unknown status
  */
int FeturaPlusAxis::getBusyState() {
    int is_busy=-1;
    size_t nread;
    bool busy, reply_check, prefix_check, checksum_check;

    // Retrieve ready vs. busy state
    pC_->buildSimpleCommand(pC_->outString_, CHECK_STATUS_CMD, sizeof(CHECK_STATUS_CMD));
    pC_->writeReadController(sizeof(CHECK_STATUS_CMD), &nread);
    if (!extractBusyState(pC_->inString_, nread, busy, reply_check, prefix_check, checksum_check)) {
        if (checksum_check) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong checksum while retrieving Fetura+ %s busy status\n", pC_->portName);
        } else if (prefix_check) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while retrieving Fetura+ %s busy status\n", pC_->portName);
        } else {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while retrieving Fetura+ %s busy status (read %zu bytes)\n", pC_->portName, nread);
        }
    } else {
        is_busy = busy;
    }

    return is_busy;
}

/** Retrieves the current zoom motor position.
  *
  * \return -1 for unknown position, otherwise 1 to 1000
  */
int FeturaPlusAxis::getCurrentPosition() {
    int readback=-1;
    size_t nread;
    bool reply_check, prefix_check, checksum_check;

    /* Retrieve readback  */
    pC_->buildSimpleCommand(pC_->outString_, READBACK_CMD, sizeof(READBACK_CMD));
    pC_->writeReadController(sizeof(READBACK_CMD), &nread);
    if (!extractReadback(pC_->inString_, nread, readback, reply_check, prefix_check, checksum_check)) {
        if (checksum_check) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong checksum while retrieving Fetura+ %s readback\n", pC_->portName);
        } else if (prefix_check) {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while retrieving Fetura+ %s readback\n", pC_->portName);
        } else {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while retrieving Fetura+ %s readback (read %zu bytes)\n", pC_->portName, nread);
        }
    }

    return readback;
}



/** Creates a new FeturaPlusController object.
  * Configuration command, called directly or from iocsh.
  *
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] asynPortName      The name of the drvAsynIPPort/drvAsynSerialPortConfigure that was created previously to connect to the Fetura+ controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  *
  * \return Always asynSuccess
  */
extern "C" int FeturaPlusCreateController(const char *portName, const char *asynPortName, int numAxes,  int movingPollPeriod, int idlePollPeriod) {
    new FeturaPlusController(portName, asynPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
    return asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg FeturaPlusCreateControllerArg0 = { "Port name", iocshArgString };
static const iocshArg FeturaPlusCreateControllerArg1 = { "Asyn port name", iocshArgString };
static const iocshArg FeturaPlusCreateControllerArg2 = { "Number of axes", iocshArgInt };
static const iocshArg FeturaPlusCreateControllerArg3 = { "Moving poll period (ms)", iocshArgInt };
static const iocshArg FeturaPlusCreateControllerArg4 = { "Idle poll period (ms)", iocshArgInt };
static const iocshArg * const FeturaPlusCreateControllerArgs[] = { &FeturaPlusCreateControllerArg0,
                                                                   &FeturaPlusCreateControllerArg1,
                                                                   &FeturaPlusCreateControllerArg2,
                                                                   &FeturaPlusCreateControllerArg3,
                                                                   &FeturaPlusCreateControllerArg4 };
static const iocshFuncDef FeturaPlusCreateControllerDef = { "FeturaPlusCreateController", 5, FeturaPlusCreateControllerArgs };
static void FeturaPlusCreateControllerCallFunc(const iocshArgBuf *args) {
    FeturaPlusCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void QioptiqFeturaPlusRegister(void) {
    iocshRegister(&FeturaPlusCreateControllerDef, FeturaPlusCreateControllerCallFunc);
}

extern "C" {
    epicsExportRegistrar(QioptiqFeturaPlusRegister);
}
