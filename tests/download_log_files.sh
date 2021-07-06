#!/usr/bin/env bash

if [ ! -d "/tmp/testlogs" ]; then
    echo "log files not found, downloading..."
    mkdir /tmp/testlogs
    URL=https://autotest.ardupilot.org/history/2021-07-06-00:07
    #wget ${URL}/HeliCopter-test.tlog --directory-prefix=/tmp/testlogs/
    #wget ${URL}/Rover-test.tlog --directory-prefix=/tmp/testlogs/
    wget ${URL}/ArduSub-test.tlog --directory-prefix=/tmp/testlogs/
    #wget ${URL}/QuadPlane-test.tlog --directory-prefix=/tmp/testlogs/
    wget ${URL}/BalanceBot-test.tlog --directory-prefix=/tmp/testlogs/
    wget ${URL}/AntennaTracker-test.tlog --directory-prefix=/tmp/testlogs/
    wget ${URL}/ArduCopter-test.tlog --directory-prefix=/tmp/testlogs/
    truncate -s 1M /tmp/testlogs/*.tlog
else
    echo "log folder already exist."
fi
