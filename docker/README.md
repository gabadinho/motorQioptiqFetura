### Dockerfile

Use this Docker configuration to generate a (non-optimized) container of Debian running EPICS 7.0.4.1 with SynApps 6.1 and the head version of this Qioptiq Fetura optics asyn motor support.

Shell scripts to help creating the container, run it, start it, are also provided.

The configuration of **qioptiqFeturaIOC** expects the controller in /dev/ttyUSB0. If this is not the case, you need to:
- Edit **ttyusb_feturacmd.sed** and replace _ttyS0_ with the correct serial connection.
- Edit **Dockerfile** and uncomment the _RUN_ command that uses the above file to patch the serial connection of the IOC.
- Edit **docker_run_feturaioc.sh** and replace _/dev/ttyUSB0_ with the correct serial connection.

