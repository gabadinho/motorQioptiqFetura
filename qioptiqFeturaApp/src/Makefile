TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += qioptiqFeturaMotor

# install qioptiqFeturaMotor.dbd into <top>/dbd
DBD += qioptiqFeturaPlusMotor.dbd

INC += QioptiqFeturaPlusMotorDriver.h

# specify all source files to be compiled and added to the library
qioptiqFeturaMotor_SRCS += QioptiqFeturaPlusMotorDriver.cpp

qioptiqFeturaMotor_LIBS += motor
qioptiqFeturaMotor_LIBS += asyn

qioptiqFeturaMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

