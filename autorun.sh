#!/bin/sh

# Simple Neutrino start script

# Neutrino's exit codes
ERROR=-1
NORMAL=0
SHUTDOWN=1
REBOOT=2
RESTART=3

echo "Starting Neutrino"

cd /tmp
/bin/neutrino >/dev/null 2>&1; RET=$?
sync

echo "Neutrino exited with exit code $RET"

if [ $RET -eq $NORMAL ]; then
	# do nothing
elif [ $RET -eq $SHUTDOWN ]; then
	poweroff
elif [ $RET -eq $REBOOT ]; then
	reboot
else # $RET -eq $ERROR
	reboot -f
fi
