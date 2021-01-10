#!../../bin/linux-x86_64/qioptiqFetura

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/qioptiqFetura.dbd"
qioptiqFetura_registerRecordDeviceDriver(pdbbase) 

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=FETURAPLUS:")

##
< feturaplus.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("FETURAPLUS:")

# Boot complete