# motorQioptiqFetura
EPICS asyn motor support for Qioptiq Fetura optics.

Please refer to the provided example IOC, qioptiqFeturaIOC, for specifics.

### Usage, in short:
1. Create serial asyn port:
	```drvAsynSerialPortConfigure("SERUSB0", "/dev/ttyUSB0", 0, 0, 0)```
2. Configure Fetura+ to above asyn port:
	```FeturaPlusCreateController("QFETPLUS", "SERUSB0", 1, 100, 1000)```
3. Load asynMotor DTYP motor record(s):
	```dbLoadTemplate("feturaplus.substitutions")```

The ```FeturaPlusCreateController``` command follows the usual API ```(portName, asynPortName, numAxes, movingPollingRate, idlePollingRate)```.

