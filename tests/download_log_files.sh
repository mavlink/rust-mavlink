if [ ! -d "/tmp/testlogs" ]; then
    echo "log files not found, downloading..."
    mkdir /tmp/testlogs
    wget http://autotest.ardupilot.org/HeliCopter-test.tlog --directory-prefix=/tmp/testlogs/
    wget http://autotest.ardupilot.org/ArduPlane-test.tlog --directory-prefix=/tmp/testlogs/
    wget http://autotest.ardupilot.org/APMrover2-test.tlog --directory-prefix=/tmp/testlogs/
    wget http://autotest.ardupilot.org/ArduSub-test.tlog --directory-prefix=/tmp/testlogs/
    wget http://autotest.ardupilot.org/QuadPlane-test.tlog --directory-prefix=/tmp/testlogs/
    wget http://autotest.ardupilot.org/BalanceBot-test.tlog --directory-prefix=/tmp/testlogs/
    wget http://autotest.ardupilot.org/AntennaTracker-test.tlog --directory-prefix=/tmp/testlogs/
    wget http://autotest.ardupilot.org/ArduCopter-test.tlog --directory-prefix=/tmp/testlogs/
    truncate -s 1M /tmp/testlogs/*.tlog
else
    echo "log folder already exist."
fi
