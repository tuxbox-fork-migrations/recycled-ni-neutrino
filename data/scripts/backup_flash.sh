#!/bin/sh

. /etc/init.d/globals

RES=""
VAR="/var/"
USRF="/var/tuxbox/config/tobackup.conf"
BAKF="/var/backup_flash.tar.gz"

TOBACKUP="/etc/auto.net*"

SHOWINFO "backup to ${BAKF} ..."

if [ -e "${USRF}" ]; then
	TOBACKUP="$TOBACKUP ${USRF}"
	while read i
		do [ "${i:0:1}" = "#" ] || TOBACKUP="$TOBACKUP ${i%%#*}"
		done < $USRF
fi

# check existence and skip files in /var
for i in $TOBACKUP; do
	if [ -e "$i" -a ${i:0:${#VAR}} != ${VAR} ]; then
		SHOWINFO "add  $i"
		RES="$RES $i"
	else
		SHOWINFO "skip $i"
	fi
done

TOBACKUP=$(echo $RES)

tar -czf "${BAKF}" $TOBACKUP 2>&1 >/dev/null

SHOWINFO "done."
