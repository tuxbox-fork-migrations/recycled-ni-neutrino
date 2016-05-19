#!/bin/sh
# -----------------------------------------------------------
# Y NI Tools (NI-Team)
# -----------------------------------------------------------
if echo "$@" | grep -q "%20"; then
	$0 ${@//%20/ }
	exit
fi

. /share/tuxbox/neutrino/httpd/scripts/_Y_Globals.sh
. /share/tuxbox/neutrino/httpd/scripts/_Y_Library.sh

BNAME=${0##*/}

PLUGIN_DIRS="/lib/tuxbox/plugins /var/tuxbox/plugins /mnt/plugins $(config_get_value_direct $y_config_neutrino 'plugin_hdd_dir')"

getLanguage()
{
	Y_LANG=$(cat /var/tuxbox/config/nhttpd.conf | grep "^Language.selected=" | cut -d= -f2)
}

file2msg() # $1 = file to show; $2 = short description; $3 = maketable or empty
{
	if [ -e $1 ]; then
		echo "ok ($1 found)"
		msg=$(cat $1)
		if [ ! "$msg" ]; then
			msg="no $2 available"
			y_format_message_html
			exit
		fi
		if [ "$3" == "maketable" ]; then
			maketable $1
		else
			y_format_message_html
		fi
	else
		echo "failed ($1 not found)"
	fi
}

maketable() # $1 = file to format
{
        # header
	echo "<div class='work_box'><div class='work_box_head'>"
	echo "<div class='work_box_head_h2'>Formatted results</div></div>"
	echo "<div class='work_box_body' style='overflow: auto; font-family: monospace'>"
	# body
	echo "<table style='border: 0 none;'>"
	while read _a _b _c _d _e _f _g _h _i _j _k _l _m _n _o _p
	do
cat << eo_tbody
	<tr>
		<td>$_a</td>
		<td>$_b</td>
		<td>$_c</td>
		<td>$_d</td>
		<td>$_e</td>
		<td>$_f</td>
		<td>$_g</td>
		<td>$_h</td>
		<td>$_i</td>
		<td>$_j</td>
		<td>$_k</td>
		<td>$_l</td>
		<td>$_m</td>
		<td>$_n</td>
		<td>$_o</td>
		<td>$_p</td>
	</tr>
eo_tbody
	done < $1
	echo "</table>"
	# footer
	echo "</div>"
}

is_mount()
{
        M=$1
        test -e "$M" && cd "$M" && M=$(pwd -P)
	while read _DEV _MTPT _FSTYPE _OPTS _REST
	do
		case "$M" in
			"netfs")
				if [ "$_FSTYPE" == "nfs" ] || [ "$_FSTYPE" == "cifs" ]; then
					RET=0; break
				else
					RET=1; continue
				fi
			;;
			"$_DEV"|"$_MTPT")
				if [ "$_FSTYPE" == "autofs" ]; then
					RET=1; continue
				else
					RET=0; break
				fi
			;;
			*)
				RET=1; continue
			;;
		esac
	done < /etc/mtab
	return $RET
}

is_exec()
{
        F=$1
	test -x /var/bin/$F
	return $?
}

is_running()
{
	D=$1
	pidof $D >/dev/null 2>&1
	return $?	
}

action=$1; shift

case "$action" in
	getline)
		FILE=$1
		LINE=${2:-1}
		if [ -f $FILE ]; then
			tmp=$(sed -ne ''"${LINE}"'p' $FILE)
			printf "%s" "$tmp"
		fi
	;;
        is_exec)
        	FILE=$1
        	is_exec $FILE && printf "%s" "true" || printf "%s" "false"
        ;;
	is_running)
		DEAMON=$1
		is_running $DEAMON && printf "%s" "true" || printf "%s" "false"
	;;
	get_update_txt)
		version="n/a"
		wget -O /tmp/NI_release.txt "http://neutrino-images.de/neutrino-images/release/release.txt"
		test -e /tmp/NI_release.txt && version=$(cat /tmp/NI_release.txt | grep ".img" | cut -d" " -f2)
		echo "version=${version// /}" > /tmp/NI_update.txt
		rm -f /tmp/NI_release.txt
	;;
	rm_update_txt)
		rm -f /tmp/NI_update.txt
	;;
	get_flash_info)
		MTPT=""
		case "$1" in
			"var")		MTPT="/var"	;;
			"root"|*)	MTPT="/"	;;
		esac
		df ${MTPT} | while read fs total used free used_percent mtpt; do
			case ${mtpt} in
				${MTPT})
					used_percent=${used_percent//\%/}
					free_percent=$(($free*100/$total))
					total=$(($total/1024))
					used=$(($used/1024))
					free=$(($free/1024))
					case $2 in
						used_percent)	printf "%d" "$used_percent";;
						free_percent)	printf "%d" "$free_percent";;
						total)		printf "%d" "$total";;
						used)		printf "%d" "$used";;
						free)		printf "%d" "$free";;
					esac
					break
				;;
			esac
		done
	;;
	get_mem_info)
		while read _desc _size _unit; do
			case $_desc in
				"MemTotal:")	total=$_size	;;
				"MemFree:")	free=$_size	;;
				"Buffers:")	buffers=$_size	;;
				"Cached:")	cached=$_size	;;
			esac
		done < /proc/meminfo
		free=$(($free+$buffers+$cached))
		used=$(($total-$free))
		#used_percent=$(($used*100/$total))
		used_percent=$((($used*200+total)/2/$total))
		free_percent=$(($free*100/$total))
		total=$(($total/1024))
		used=$(($used/1024))
		free=$(($free/1024))
		case $1 in
			used_percent)	printf "%d" "$used_percent";;
			free_percent)	printf "%d" "$free_percent";;
			total)		printf "%d" "$total";;
			used)		printf "%d" "$used";;
			free)		printf "%d" "$free";;
		esac
	;;
	get_mtd_info)
		while read dev size erasesize name; do
			case ${dev:0:3} in
				mtd)
					test "$1" = "dev" && printf "%s<br/>" "$dev"
					test "$1" = "name" && printf "%s<br/>" "${name//\"/}"
				;;
			esac
		done < /proc/mtd
	;;
	get_cpu_info)
		for i in 1 2; do
			while read _cpu _user _nice _system _idle _rest; do
				case $_cpu in
					cpu)
						case $i in
							1)
								fst_all=$(($_user + $_nice + $_system + $_idle))
								fst_idle=$_idle
							;;
							2)
								snd_all=$(($_user + $_nice + $_system + $_idle))
								snd_idle=$_idle
							;;
						esac
					;;
				esac
			done < /proc/stat
			test $i = 1 && sleep 1
		done

		diff_all=$(($snd_all - $fst_all))
		diff_idle=$(($snd_idle - $fst_idle))
		_idle_percent=$(($diff_idle * 100 / $diff_all))
		_used_percent=$((100 - $_idle_percent))

		case $1 in
			used_percent)	printf "%s" "$_used_percent";;
		esac
	;;
	# camd-control
	#softcamstop)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh stop;;
	#softcamstart)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh start;;
	softcamreset)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh reset;;
	mgcamdstop)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh mgcamd_stop;;  
	mgcamdstart)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh mgcamd_start;;
	mgcamdreset)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh mgcamd_reset;;
	gboxstop)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh gbox_stop;;
	gboxstart)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh gbox_start;;
	gboxreset)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh gbox_reset;;
	cs2gboxstop)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh cs2gbox_stop;;
	cs2gboxstart)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh cs2gbox_start;;
	cs2gboxreset)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh cs2gbox_reset;;
	osemustop)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh osemu_stop;;
	osemustart)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh osemu_start;;
	osemureset)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh osemu_reset;;
	oscamstop)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh oscam_stop;;
	oscamstart)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh oscam_start;;
	oscamreset)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh oscam_reset;;
	doscamstop)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh doscam_stop;;
	doscamstart)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh doscam_start;;
	doscamreset)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh doscam_reset;;
	newcsstop)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh newcs_stop;;
	newcsstart)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh newcs_start;;
	newcsreset)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Camd-control.sh newcs_reset;;

	vinfo)
	        if ! [ -x /var/bin/$1 ]; then
	        	printf "%s" "n/a"
	        	exit
	        fi
		CAM=$(echo $1 | tr [:lower:] [:upper:])
		if [ "$CAM" = "GBOX" ]; then
			CAM=GBOX.NET
		fi
		V=$(/bin/vinfo $CAM /var/bin/$1)
		printf "%s" "${V//keine Informationen gefunden/}"
	;;

	# syntax	file2msg file "short-description" (maketable)
	mgshareinfo)	file2msg /tmp/mgshare.info "n-lines" maketable;;
	mginfo)		file2msg /tmp/mg.info "cs-info" maketable;;
	cccamdinfo)	file2msg /tmp/cccamd.info "c-lines" maketable;;
	cs2gboxinfo)	file2msg /tmp/cs2gbox.info "cs2gbox-info";;
	shareonl)	file2msg /tmp/share.onl "online-share-info" maketable;;
	shareinfo)	file2msg /tmp/share.info "cs-info" maketable;;
	sc01info)	file2msg /tmp/sc01.info "slot1-info";;
	sc02info)	file2msg /tmp/sc02.info "slot2-info";;
	ecminfo)	file2msg /tmp/ecm.info "ecm-info";;
	pidinfo)	file2msg /tmp/pid.info "pid-info";;
	oscamversion)	file2msg /tmp/.oscam/oscam.version "oscam-info";;
	doscamversion)	file2msg /tmp/.doscam/doscam.version "doscam-info";;

	get_oscam_webif_port)
		if [ -e /var/tuxbox/config/oscam.conf ]; then
			_port=$(grep -m 1 -i "^[:space:]*httpport" /var/tuxbox/config/oscam.conf | cut -d'=' -f2)
			_port=$(echo $_port | dos2unix -u)
		fi
		printf "%s" ${_port:-8080}
	;;

	get_doscam_webif_port)
		if [ -e /var/tuxbox/config/doscam.cfg ]; then
			_port=$(grep -m 1 -i "^[:space:]*httpport" /var/tuxbox/config/doscam.cfg | cut -d'=' -f2)
			_port=$(echo $_port | dos2unix -u)
		fi
		printf "%s" ${_port:-8080}
	;;

	get_xupnpd_webif_port)
		if [ -e /share/xupnpd/xupnpd.lua ]; then
			_port=$(grep -m 1 "^[:space:]*cfg.http_port" /share/xupnpd/xupnpd.lua | cut -d'=' -f2)
			_port=$(echo $_port | dos2unix -u)
		fi
		printf "%s" $_port
	;;

	countcards)
		getLanguage
		if [ "$Y_LANG" == "Deutsch" ]
		then
			Y_L_count="Karten sind online."
			Y_L_recount="können weitergegeben werden."
		else
			Y_L_count="cards online."
			Y_L_recount="can be shared."
		fi
		FILE=$1
		COUNT=0
		_DIST=""
		_LEV=""
		if [ -f $FILE ]; then
			COUNT=$(cat $FILE | wc -l | sed 's/^ *//')
			echo "<b>$COUNT</b> $Y_L_count"
			test $COUNT = 0 && exit

			RECOUNT=$(cat $FILE | grep -v Lev:0 | wc -l | sed 's/^ *//')
			echo "<b>$RECOUNT</b> $Y_L_recount"

			for i in $(seq 0 9); do
				DIST=$(cat $FILE | grep -H dist:$i | wc -l | sed 's/^ *//')
				test $DIST = 0 || _DIST=$(echo -e "$_DIST\n\tDist. $i: $DIST")
			done
			for i in $(seq 0 9); do
				LEV=$(cat $FILE | grep -H Lev:$i | wc -l | sed 's/^ *//')
				test $LEV = 0 || _LEV=$(echo -e "$_LEV\n\tLevel $i: $LEV")
			done

			echo "<pre>$_LEV<br/>$_DIST</pre>"
		else
			echo "failed ($FILE not found)"
			exit
		fi

		test $COUNT = 0 && echo "Es sind derzeit keine Karten online!"
	;;

	# zapit-control
	resolution)		pzapit --${1}		&& echo "ok" || echo "failed";;
	43mode)			pzapit -vm43 ${1}	&& echo "ok" || echo "failed";;
	reload_channellists)    pzapit -c		&& echo "ok" || echo "failed";;
	reset_tuner)				E=0
				pzapit -esb;	E=$(($E+$?));	sleep 1
				pzapit -lsb;	E=$(($E+$?));	sleep 1
				pzapit -rz;	E=$(($E+$?))
				test $E = 0 		&& echo "ok" || echo "failed"
	;;
	# netfs-control
	is_mount)
		MTPT=$1
		is_mount $MTPT && printf "%s" "true" || printf "%s" "false"
	;;
	do_mount_all)
		msg=$(/etc/init.d/fstab start_netfs)
		y_format_message_html
	;;
	do_umount_all)
		msg=$(/etc/init.d/fstab stop_netfs)
		y_format_message_html
	;;
	do_mount)
	        MTPT=$1
	        test $MTPT || exit 1
		test -d $MTPT || mkdir -p $MTPT;
		FLAG="/var/etc/.srv"
		if OUT=$(mount $MTPT 2>&1 >/dev/null); then
			RET=$?
			msg="mount: $MTPT - success ($RET)"
			test -e $FLAG || touch $FLAG
		else
			RET=$?
			msg="mount: $MTPT - failed ($RET)<br>$OUT"
		fi
		y_format_message_html
	;;
	do_umount)
	        MTPT=$1
	        test $MTPT || exit 1
		FLAG="/var/etc/.srv"
		rm -f $FLAG
		if OUT=$(umount $MTPT 2>&1 >/dev/null); then
			RET=$?
			msg="umount: $MTPT - success ($RET)"
		else
			RET=$?
			msg="umount: $MTPT - failed ($RET)<br>$OUT"
		fi
		is_mount netfs && touch $FLAG
		y_format_message_html
	;;
	# automounter-control
	do_autofs)
		case $1 in
			start|stop|restart|reload)
				msg=$(service autofs $1)
				y_format_message_html
			;;
		esac
	;;
	# infoicons-control
	rm_icons_flag)
		while [ $1 ]; do
			if [ -e $1 ]; then
				rm -f $1 && echo "ok ($1 removed)<br />" || echo "failed (can't remove $1)<br />"
			else
				echo "nothing to do ($1 not found)<br />"
			fi
			shift
		done
	;;
	touch_icons_flag)
		while [ $1 ]; do
			if [ ! -e $1 ]; then
				touch $1 && echo "ok ($1 created)<br />" || echo "failed (can't create $1)<br />"
			else
				echo "nothing to do ($1 always exist)<br />"
			fi
			shift
		done
	;;
	read_icons_flag)
		test -z $1 && exit 1
		test -e $1 && S=$(cat $1)
		printf "%s" "$S"
	;;
	# plugin-control
	p_fcm_start)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh fcm_start;;
	p_fcm_stop)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh fcm_stop;;
	p_nfs_start)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh nfs_start;;
	p_nfs_stop)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh nfs_stop;;
	p_smb_start)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh smb_start;;
	p_smb_stop)	/share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh smb_stop;;
	p_txc_start)    /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh txc_start;;
	p_txc_stop)     /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh txc_stop;;
	p_txm_start)    /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh txm_start;;
	p_txm_stop)     /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh txm_stop;;
	p_ina_start)    /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh ina_start;;
	p_ina_stop)     /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh ina_stop;;
	p_drop_start)   /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh drop_start;;
	p_drop_stop)    /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh drop_stop;;
	p_ush_start)    /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh ush_start;;
	p_ush_stop)     /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh ush_stop;;
	p_djm_start)    /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh djm_start;;
	p_djm_stop)     /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh djm_stop;;
	p_xud_start)    /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh xud_start;;
	p_xud_stop)     /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh xud_stop;;
	p_cro_start)    /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh cro_start;;
	p_cro_stop)     /share/tuxbox/neutrino/httpd/scripts/Y_NI_Plugin-control.sh cro_stop;;
	# plugins on blue-button
	p_show)
		PLUGIN=$1
		for PLUGIN_DIR in $PLUGIN_DIRS
		do
			cd $PLUGIN_DIR
			test -w "$PLUGIN" || continue
			echo "[$BNAME] modifying $PLUGIN_DIR/$PLUGIN"
			sed -i "s/^\(hide=\).*$/\1"0"/" $PLUGIN
		done
	;;
	p_hide)
		PLUGIN=$1
		for PLUGIN_DIR in $PLUGIN_DIRS
		do
			cd $PLUGIN_DIR
			test -w "$PLUGIN" || continue
			echo "[$BNAME] modifying $PLUGIN_DIR/$PLUGIN"
			# fix for missing trailing linefeed in cfg-file
			test "$(tail -c1 $PLUGIN)" != "" && echo "" >> $PLUGIN
			if grep -q "^hide=" $PLUGIN
			then
				sed -i "s/^\(hide=\).*$/\1"1"/" $PLUGIN
			else
				echo "hide=1" >> $PLUGIN
			fi
		done
	;;
	p_list)
		getLanguage
		if [ "$Y_LANG" == "Deutsch" ]
		then
			Y_L_show="Anzeigen"
			Y_L_hide="Verstecken"
		else
			Y_L_show="Show"
			Y_L_hide="Hide"
		fi
		for PLUGIN_DIR in $PLUGIN_DIRS
		do
			test -e $PLUGIN_DIR || continue
			cd $PLUGIN_DIR
			PLUGINS=$(ls -1 *.cfg 2>/dev/null)
			for PLUGIN in $PLUGINS
			do
				if [ "$1" == "lua" ]
				then
					# lua-plugins don't need the executable flag
					test -e ${PLUGIN%%.*}.${1} || continue
				else
					test -x ${PLUGIN%%.*}.${1} || continue
				fi
				NAME=$(cat $PLUGIN | grep "^name=" | cut -d= -f2)
				HIDE=$(cat $PLUGIN | grep "^hide=" | cut -d= -f2)
				case $HIDE in
					1)
						IMG="<img src='images/x_red.png' class='status'>"
						INP="<input type='button' value='$Y_L_show' onclick='Y_NI_Tools(\"p_show $PLUGIN\", 1000);' />"
					;;
					*)
						IMG="<img src='images/check_green.png' class='status'>"
						INP="<input type='button' value='$Y_L_hide' onclick='Y_NI_Tools(\"p_hide $PLUGIN\", 1000);' />"
					;;
				esac
cat << eoPLUGIN
			<!-- $NAME -->
			<tr class="list">
				<td>
					$IMG
					<span title="$PLUGIN_DIR/${PLUGIN%%.*}.${1}">
						$NAME
					</span>
				</td>
				<td>
					$INP
				</td>
			</tr>
eoPLUGIN
			done
		done
	;;
	*)
		echo "[$BNAME] Parameter wrong: $action $*"
	;;
esac
