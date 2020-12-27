TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = qioptiqFetura
# qioptiqFetura.dbd will be created and installed
DBD += qioptiqFetura.dbd

# qioptiqFetura.dbd will be made up from these files:
qioptiqFetura_DBD += base.dbd

# Include dbd files from all support applications:
#qioptiqFetura_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
#qioptiqFetura_LIBS += xxx

# qioptiqFetura_registerRecordDeviceDriver.cpp derives from qioptiqFetura.dbd
qioptiqFetura_SRCS += qioptiqFetura_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
qioptiqFetura_SRCS_DEFAULT += qioptiqFeturaMain.cpp
qioptiqFetura_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#qioptiqFetura_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
qioptiqFetura_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

