#!../../bin/linux-x86_64/gtest

#- You may have to change gtest to something else
#- everywhere it appears in this file

#< envPaths

## Register all support components
dbLoadDatabase("../../dbd/gtest.dbd",0,0)
gtest_registerRecordDeviceDriver(pdbbase) 

## Load record instances
dbLoadRecords("../../db/gtest.db","user=gabadinho")

iocInit()

## Start any sequence programs
#seq sncgtest,"user=gabadinho"
