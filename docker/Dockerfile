# Base image
FROM amd64/debian:stable

# Copy sed-patch files into image
COPY *.sed /tmp/

# Package dependencies
RUN apt-get update && \
    apt-get install -y \
        build-essential \
        libreadline-dev \
        wget \
        re2c \
        vim \
        git-all

# Download EPICS, unpack, build it
RUN wget -P /tmp/ https://epics-controls.org/download/base/base-7.0.4.1.tar.gz && \
    mkdir -p /usr/local/epics && \
    tar xfz /tmp/base-7.0.4.1.tar.gz -C /usr/local/epics && \
    ln -s /usr/local/epics/base-R7.0.4.1 /usr/local/epics/base && \
    cd /usr/local/epics/base && make

# Download synApps, unpack, patch configuration, build it
RUN wget -P /tmp/ https://epics.anl.gov/bcda/synApps/tar/synApps_6_1.tar.gz && \
    mkdir -p /usr/local/epics/modules && \
    tar xfz /tmp/synApps_6_1.tar.gz -C /usr/local/epics/modules && \
    ln -s /usr/local/epics/modules/synApps_6_1 /usr/local/epics/modules/synApps && \
    sed -i -f /tmp/synapps_release.sed /usr/local/epics/modules/synApps/support/configure/RELEASE && \
    cd /usr/local/epics/modules/synApps/support && make release && \
    sed -i -f /tmp/ipac_release.sed /usr/local/epics/modules/synApps/support/asyn-R4-36/configure/RELEASE && \
    sed -i -f /tmp/iocstats_release.sed /usr/local/epics/modules/synApps/support//iocStats-3-1-16/configure/RELEASE && \
    sed -i -f /tmp/ipac_release.sed /usr/local/epics/modules/synApps/support/motor-R7-1/configure/RELEASE && \
    cd /usr/local/epics/modules/synApps/support && make

# Download latest Qioptiq Fetura driver, patch configuration, build it and its example IOC
RUN cd /usr/local/epics && git clone https://github.com/gabadinho/motorQioptiqFetura.git && \
    sed -i -f /tmp/epicsbase_release.sed /usr/local/epics/motorQioptiqFetura/configure/RELEASE && \
    cp /usr/local/epics/motorQioptiqFetura/configure/EXAMPLE_RELEASE.local /usr/local/epics/motorQioptiqFetura/configure/RELEASE.local && \
    sed -i -f /tmp/releaselocal.sed /usr/local/epics/motorQioptiqFetura/configure/RELEASE.local

# Patch serial device connection
# RUN sed -i -f /tmp/ttyusb_feturacmd.sed /usr/local/epics/motorQioptiqFetura/iocs/qioptiqFeturaIOC/iocBoot/iocQioptiqFetura/feturaplus.cmd

# Build driver and IOC
RUN cd /usr/local/epics/motorQioptiqFetura && make && \
    cd /usr/local/epics/motorQioptiqFetura/iocs/qioptiqFeturaIOC && make

# Start IOC
CMD cd /usr/local/epics/motorQioptiqFetura/iocs/qioptiqFeturaIOC/iocBoot/iocQioptiqFetura && ../../bin/linux-x86_64/qioptiqFetura st.cmd
# CMD ["/bin/bash"]

