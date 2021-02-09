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

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <asynMotorController.h>
#include <asynMotorAxis.h>

#include <epicsExport.h>

#include "QioptiqFeturaPlusMotorDriver.h"



#define FETURAPLUS_CONTROLLER_TIMEOUT 0.11

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
                         asynUInt32DigitalMask,
                         asynUInt32DigitalMask,
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
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: cannot connect to fetura+ %s controller\n", driverName, functionName, portName);
    } else {
        pasynOctetSyncIO->setInputEos(pasynUserController_, "", 0);
        pasynOctetSyncIO->setOutputEos(pasynUserController_, "", 0);

        memcpy(this->outString_, SYNC_CMD, sizeof(SYNC_CMD));
        this->writeReadController(sizeof(SYNC_CMD), &nread);
        if (nread == sizeof(SYNC_REPLY)) {
            if (!memcmp(this->inString_, SYNC_REPLY, sizeof(SYNC_REPLY))) {
            } else {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: unable to synchronize with fetura+ %s controller\n", driverName, functionName, portName);
            }
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: unable to synchronize with fetura+ %s controller\n", driverName, functionName, portName);
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
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  */
void FeturaPlusController::report(FILE *fp, int level) {
    size_t nread;
    unsigned serialnr=0, firmlo=0, firmhi=0;

    fprintf(fp, "Qioptiq Fetura+ optics controller %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

    if (level > 0) {
        // Retrieve serial number
        memcpy(this->outString_, SERIALNUMBER_CMD, sizeof(SERIALNUMBER_CMD));
        this->writeReadController(sizeof(SERIALNUMBER_CMD), &nread);
        if (nread == sizeof(SERIALNUMBER_REPLY_PREFIX)+5) {
            if (!memcmp(this->inString_, SERIALNUMBER_REPLY_PREFIX, sizeof(SERIALNUMBER_REPLY_PREFIX))) {
                if (FeturaPlusAxis::checkChecksumAtEnd(this->inString_,nread)) {
                    serialnr = (unsigned char)(this->inString_[sizeof(SERIALNUMBER_REPLY_PREFIX)] << 24) + 
                               (unsigned char)(this->inString_[sizeof(SERIALNUMBER_REPLY_PREFIX)+1] << 16) + 
                               (unsigned char)(this->inString_[sizeof(SERIALNUMBER_REPLY_PREFIX)+2] << 8) + 
                               (unsigned char)(this->inString_[sizeof(SERIALNUMBER_REPLY_PREFIX)+3]);
                } else {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong checksum while getting fetura+ %s serial number\n", this->portName);
                }
            } else {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while getting fetura+ %s serial number\n", this->portName);
            }
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while getting fetura+ %s serial number (read %zu bytes)\n", this->portName, nread);
        }
        if (serialnr) {
            fprintf(fp, "  serial number=%X\n", serialnr);
        } else {
            fprintf(fp, "  unknown serial number\n");
        }

        // Retrieve firmware version
        memcpy(this->outString_, FIRMWARE_CMD, sizeof(FIRMWARE_CMD));
        this->writeReadController(sizeof(FIRMWARE_CMD), &nread);
        if (nread == sizeof(FIRMWARE_REPLY_PREFIX)+5) {
            if (!memcmp(this->inString_, FIRMWARE_REPLY_PREFIX, sizeof(FIRMWARE_REPLY_PREFIX))) {
                if (FeturaPlusAxis::checkChecksumAtEnd(this->inString_, nread)) {
                    firmhi = (unsigned char)(this->inString_[sizeof(FIRMWARE_REPLY_PREFIX)+2] << 8) + 
                             (unsigned char)(this->inString_[sizeof(FIRMWARE_REPLY_PREFIX)+3]);
                    firmlo = (unsigned char)(this->inString_[sizeof(FIRMWARE_REPLY_PREFIX)] << 8) + 
                             (unsigned char)(this->inString_[sizeof(FIRMWARE_REPLY_PREFIX)+1]);
                } else {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong checksum while getting fetura+ %s firmware version\n", this->portName);
                }
            } else {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while getting fetura+ %s firmware version\n", this->portName);
            }
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while getting fetura+ %s firmware version (read %zu bytes)\n", this->portName, nread);
        }
        if ((firmhi) || (firmlo)) {
            fprintf(fp, "  firmware version=%d.%d\n", firmhi, firmlo);
        } else {
            fprintf(fp, "  unknown firmware version\n");
        }

    }

    // Call the base class method
    asynMotorController::report(fp, level);
}

/** Returns a pointer to an FeturaPlusAxis object.
  *
  * \param[in] pasynUser asynUser structure that encodes the axis index number.
  *
  * \return FeturaPlusAxis object or NULL if the axis number encoded in pasynUser is invalid
  */
FeturaPlusAxis* FeturaPlusController::getAxis(asynUser *pasynUser) {
    return static_cast<FeturaPlusAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an FeturaPlusAxis object.
  *
  * \param[in] axisNo Axis index number.
  *
  * \return FeturaPlusAxis object or NULL if the axis number encoded in pasynUser is invalid
  */
FeturaPlusAxis* FeturaPlusController::getAxis(int axisNo) {
    return static_cast<FeturaPlusAxis*>(asynMotorController::getAxis(axisNo));
}

/** Wrapper of writeReadController with explicit output length, avoiding usage of strlen().
  *
  * \param[in] nwrite Number of bytes to write
  *
  * \param[out] nread Number of bytes read
  */
asynStatus FeturaPlusController::writeReadController(size_t nwrite, size_t *nread) {
    size_t nwrote;
    int eomReason;

    *nread = 0;
    return pasynOctetSyncIO->writeRead(pasynUserController_,
                                       outString_, nwrite,
                                       inString_, sizeof(inString_),
                                       FETURAPLUS_CONTROLLER_TIMEOUT,
                                       &nwrote, nread, &eomReason);
}



// These are the FeturaPlusAxis methods

/** Creates a new FeturaPlusAxis object.
  *
  * \param[in] pC Pointer to the FeturaPlusController to which this axis belongs
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1
  */
FeturaPlusAxis::FeturaPlusAxis(FeturaPlusController *pC, int axisNo): asynMotorAxis(pC, axisNo), pC_(pC) {
    int home_done=-1;

    home_done = getHomedState();
    if (home_done!=-1) {
        this->setIntegerParam(pC_->motorStatusHomed_, home_done);
    }

    callParamCallbacks();
}

/** Reports on status of the driver.
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorAxis::report()
  *
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  */
void FeturaPlusAxis::report(FILE *fp, int level) {
    int home_done=-1;

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
    size_t nread;
    int errors=0, moveto_pos=position, is_busy;
    unsigned char checksum;

    // First check if ready
    is_busy = getBusyState();
    if (is_busy == 0) {
        setIntegerParam(pC_->motorStatusDone_, 0);

        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Moving fetura+ %s to %f (at %f)\n", pC_->portName, position, maxVelocity);
        if (maxVelocity > MOVEMODE_THRESHOLD) {
            position += MOVEMODE_THRESHOLD;
        }

        memcpy(pC_->outString_, MOVETO_CMD_PREFIX, sizeof(MOVETO_CMD_PREFIX));
        pC_->outString_[sizeof(MOVETO_CMD_PREFIX)] = (moveto_pos & 0xFF00)>>8;
        pC_->outString_[sizeof(MOVETO_CMD_PREFIX)+1] = (moveto_pos & 0xFF);
        checksum = calculateChecksum(pC_->outString_, sizeof(MOVETO_CMD_PREFIX)+2);
        pC_->outString_[sizeof(MOVETO_CMD_PREFIX)+2] = checksum;

        pC_->writeReadController(sizeof(MOVETO_CMD_PREFIX)+3, &nread);
        if ((nread == sizeof(ACKNOWLEDGE)) || (nread == sizeof(READBACK_REPLY_PREFIX)+3)) {
            if (nread != sizeof(ACKNOWLEDGE)) {
                if ((!memcmp(pC_->inString_, READBACK_REPLY_PREFIX, sizeof(READBACK_REPLY_PREFIX))) && (pC_->inString_[sizeof(READBACK_REPLY_PREFIX)] == 0x00) &&
                    ((pC_->inString_[sizeof(READBACK_REPLY_PREFIX)+1] == 0x00) || (pC_->inString_[sizeof(READBACK_REPLY_PREFIX)+1] == 0x01))) {
                    if (checkChecksumAtEnd(pC_->inString_, nread)) {
                        errors = 1-pC_->inString_[sizeof(READBACK_REPLY_PREFIX)+1];
                    } else {
                        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong checksum while moving fetura+ %s\n", pC_->portName);
                        errors++;
                    }
                }
            } else {
                if (!memcmp(pC_->inString_, ACKNOWLEDGE, sizeof(ACKNOWLEDGE))) {
                } else {
                    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Unrecognized reply while moving fetura+ %s\n", pC_->portName);
                    errors++;
                }
            }
        } else {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Unrecognized reply while moving fetura+ %s\n", pC_->portName);
            errors++;
        }
    } else {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Cannot move fetura+ %s because device is busy\n", pC_->portName);
        errors++;
    }

    if (!errors) {
        status = asynSuccess;
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
    int errors=0, is_busy;

    // First check if ready
    is_busy = getBusyState();
    if (is_busy == 0) {
        setIntegerParam(pC_->motorStatusDone_, 0);
        setDoubleParam(pC_->motorPosition_, 0);

        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Homing fetura+ %s...\n", pC_->portName);

        // Reset controller
        memcpy(pC_->outString_, RESET_CMD, sizeof(RESET_CMD));
        pC_->writeReadController(sizeof(RESET_CMD), &nread);
        if (nread == sizeof(ACKNOWLEDGE)) {
            if (!memcmp(pC_->inString_, ACKNOWLEDGE, sizeof(ACKNOWLEDGE))) {
                // Wait and flush communications
                epicsThreadSleep(0.501);
                pasynOctetSyncIO->flush(pC_->pasynUserController_);
            } else {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Unrecognized reply while homing fetura+ %s\n", pC_->portName);
                errors++;
            }
        } else {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while homing fetura+ %s (read %zu bytes)\n", pC_->portName, nread);
            errors++;
        }
    } else {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_FLOW, "Cannot home fetura+ %s because device is busy\n", pC_->portName);
        errors++;
    }

    if (!errors) {
        status = asynSuccess;
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
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
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

/** Raises the motor record problem status.
  *
  * \param[in] status Last operation status
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

    // Retrieve homing vs. homed
    memcpy(pC_->outString_, CHECKHOME_CMD, sizeof(CHECKHOME_CMD));
    pC_->writeReadController(sizeof(CHECKHOME_CMD), &nread);
    if (nread == sizeof(CHECKHOME_REPLY_PREFIX)+3) {
        if (!memcmp(pC_->inString_, CHECKHOME_REPLY_PREFIX, sizeof(CHECKHOME_REPLY_PREFIX))) {
            if ((pC_->inString_[sizeof(CHECKHOME_REPLY_PREFIX)] == 0x00) && 
                ((pC_->inString_[sizeof(CHECKHOME_REPLY_PREFIX)+1] == 0x00) || (pC_->inString_[sizeof(CHECKHOME_REPLY_PREFIX)+1] == 0x01))) {
                if (checkChecksumAtEnd(pC_->inString_, nread)) {
                    home_done = pC_->inString_[sizeof(CHECKHOME_REPLY_PREFIX)+1];
                } else {
                    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong checksum while retrieving fetura+ %s homing status\n", pC_->portName);
                }
            } else {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while retrieving fetura+ %s homing status\n", pC_->portName);
            }
        } else {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while retrieving fetura+ %s homing status\n", pC_->portName);
        }
    } else {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while retrieving fetura+ %s homing status (read %zu bytes)\n", pC_->portName, nread);
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

    // Retrieve ready vs. busy state
    memcpy(pC_->outString_, CHECK_STATUS_CMD, sizeof(CHECK_STATUS_CMD));
    pC_->writeReadController(sizeof(CHECK_STATUS_CMD), &nread);
    if (nread == sizeof(STATUS_REPLY_PREFIX)+3) {
        if (!memcmp(pC_->inString_, STATUS_REPLY_PREFIX, sizeof(STATUS_REPLY_PREFIX))) {
            if ((pC_->inString_[sizeof(STATUS_REPLY_PREFIX)] == 0x00) && 
                ((pC_->inString_[sizeof(STATUS_REPLY_PREFIX)+1] == 0x00) || (pC_->inString_[sizeof(STATUS_REPLY_PREFIX)+1] == 0x01))) {
                if (checkChecksumAtEnd(pC_->inString_, nread)) {
                    is_busy = pC_->inString_[sizeof(STATUS_REPLY_PREFIX)+1];
                } else {
                    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong checksum while retrieving fetura+ %s status\n", pC_->portName);
                }
            } else {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Unrecognized reply while retrieving fetura+ %s status\n", pC_->portName);
            }
        } else {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Unrecognized reply while retrieving fetura+ %s status\n", pC_->portName);
        }
    } else {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while retrieving fetura+ %s status (read %zu bytes)\n", pC_->portName, nread);
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

    /* Retrieve readback  */
    memcpy(pC_->outString_, READBACK_CMD, sizeof(READBACK_CMD));
    pC_->writeReadController(sizeof(READBACK_CMD), &nread);
    if (nread == sizeof(READBACK_REPLY_PREFIX)+3) {
        if (!memcmp(pC_->inString_, READBACK_REPLY_PREFIX, sizeof(READBACK_REPLY_PREFIX))) {
            if (checkChecksumAtEnd(pC_->inString_, nread)) {
                readback = pC_->inString_[sizeof(READBACK_REPLY_PREFIX)] << 8;
                readback += (unsigned char)(pC_->inString_[sizeof(READBACK_REPLY_PREFIX)+1]);
            } else {
                asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong checksum while retrieving fetura+ %s readback\n", pC_->portName);
            }
        } else {
            asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Unrecognized reply while retrieving fetura+ %s readback\n", pC_->portName);
        }
    } else {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Wrong answer while retrieving fetura+ %s readback (read %zu bytes)\n", pC_->portName, nread);
    }

    return readback;
}

unsigned char FeturaPlusAxis::calculateChecksum(char *frame, size_t frame_length) {
    unsigned int sum=0;

    for (size_t i=0; i<frame_length; i++) {
        sum += (unsigned char)(frame[i]);
    }
    sum = sum & 0xFF;
    return sum;
}

bool FeturaPlusAxis::checkChecksumAtEnd(char *frame, size_t frame_length) {
    unsigned char calculated_checksum = calculateChecksum(frame+1, frame_length-2);
    unsigned char given_checksum = (unsigned char)(frame[frame_length-1]);
    return (calculated_checksum==given_checksum);
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
