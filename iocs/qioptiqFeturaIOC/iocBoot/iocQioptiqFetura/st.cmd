#!../../bin/linux-x86_64/qioptiqFetura

#- You may have to change qioptiqFetura to something else
#- everywhere it appears in this file

#< envPaths

## Register all support components
dbLoadDatabase("../../dbd/qioptiqFetura.dbd",0,0)
qioptiqFetura_registerRecordDeviceDriver(pdbbase) 

## Load record instances
dbLoadRecords("../../db/qioptiqFetura.db","user=gabadinho")

iocInit()

## Start any sequence programs
#seq sncqioptiqFetura,"user=gabadinho"
