/*
	Neutrino-GUI  -   DBoxII-Project

	Copyright (C) 2001 Steffen Hehn 'McClean'
	Copyright (C) 2010-2012 Stefan Seyfried

	License: GPL

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <dlfcn.h>
#include <sys/mount.h>

#include <global.h>
#include <neutrino.h>
#include <neutrino_menue.h>
#include "hdd_menu.h"

#include <gui/widget/icons.h>
#include <gui/widget/stringinput.h>
#include <gui/widget/messagebox.h>
#include <gui/widget/hintbox.h>
#include <gui/widget/progresswindow.h>

#include <system/helpers.h>
#include <system/settings.h>
#include <system/debug.h>

#include <mymenu.h>
#include <driver/screen_max.h>

#define HDD_NOISE_OPTION_COUNT 4
const CMenuOptionChooser::keyval HDD_NOISE_OPTIONS[HDD_NOISE_OPTION_COUNT] =
{
	{ 0,   LOCALE_OPTIONS_OFF },
	{ 128, LOCALE_HDD_SLOW },
	{ 190, LOCALE_HDD_MIDDLE },
	{ 254, LOCALE_HDD_FAST }
};

#define HDD_FILESYS_OPTION_COUNT 3
const CMenuOptionChooser::keyval HDD_FILESYS_OPTIONS[HDD_FILESYS_OPTION_COUNT] =
{
	{ 0, LOCALE_HDD_EXT3 },
	{ 1, LOCALE_HDD_REISER },
	{ 2, LOCALE_OPTIONS_OFF }
};
#define HDD_SLEEP_OPTION_COUNT 7
const CMenuOptionChooser::keyval HDD_SLEEP_OPTIONS[HDD_SLEEP_OPTION_COUNT] =
{
	{ 0,   LOCALE_OPTIONS_OFF },
	{ 12,  LOCALE_HDD_1MIN },
	{ 60,  LOCALE_HDD_5MIN },
	{ 120, LOCALE_HDD_10MIN },
	{ 240, LOCALE_HDD_20MIN },
	{ 241, LOCALE_HDD_30MIN },
	{ 242, LOCALE_HDD_60MIN }
};

static int my_filter(const struct dirent * dent)
{
	if(dent->d_name[0] == 's' && dent->d_name[1] == 'd')
		return 1;
	return 0;
}

CHDDMenuHandler::CHDDMenuHandler()
{
	width = w_max (58, 10);
}

CHDDMenuHandler::~CHDDMenuHandler()
{

}

int CHDDMenuHandler::exec(CMenuTarget* parent, const std::string &/*actionkey*/)
{
	if (parent)
		parent->hide();

	return doMenu ();
}

int CHDDMenuHandler::doMenu ()
{
	FILE * f;
	int fd;
	struct dirent **namelist;
	int ret;
	struct stat s;
	int root_dev = -1;

	bool hdd_found = 0;
	int n = scandir("/sys/block", &namelist, my_filter, alphasort);

	if (n < 0) {
		perror("CHDDMenuHandler::doMenu: scandir(\"/sys/block\") failed");
		return menu_return::RETURN_REPAINT;
	}


	CMenuWidget* hddmenu = new CMenuWidget(LOCALE_MAINMENU_SETTINGS, NEUTRINO_ICON_SETTINGS, width, MN_WIDGET_ID_DRIVESETUP);

	//if no drives found, select 'back'
	if (hdd_found == 0 && hddmenu->getSelected() != -1)
		hddmenu->setSelected(2);

	hddmenu->addIntroItems(LOCALE_HDD_SETTINGS);
	CHDDFmtExec fmtexec;
	CHDDChkExec chkexec;

	CHDDDestExec hddexec;
	CMenuForwarder * mf = new CMenuForwarder(LOCALE_HDD_ACTIVATE, true, "", &hddexec, NULL, CRCInput::RC_red,NEUTRINO_ICON_BUTTON_RED);
	mf->setHint("", LOCALE_MENU_HINT_HDD_APPLY);
	hddmenu->addItem(mf);

	hddmenu->addItem(new CMenuSeparator(CMenuSeparator::LINE | CMenuSeparator::STRING, LOCALE_HDD_EXTENDED_SETTINGS));

	CMenuOptionChooser * mc = new CMenuOptionChooser(LOCALE_HDD_SLEEP, &g_settings.hdd_sleep, HDD_SLEEP_OPTIONS, HDD_SLEEP_OPTION_COUNT, true);
	mc->setHint("", LOCALE_MENU_HINT_HDD_SLEEP);
	hddmenu->addItem(mc);

	mc = new CMenuOptionChooser(LOCALE_HDD_NOISE, &g_settings.hdd_noise, HDD_NOISE_OPTIONS, HDD_NOISE_OPTION_COUNT, true);
	mc->setHint("", LOCALE_MENU_HINT_HDD_NOISE);
	hddmenu->addItem(mc);

	//if(n > 0)
	hddmenu->addItem(new CMenuSeparator(CMenuSeparator::LINE | CMenuSeparator::STRING, LOCALE_HDD_MANAGE));

	ret = stat("/", &s);
	if (ret != -1)
		root_dev = (s.st_dev & 0x0ffc0); /* hda = 0x0300, hdb = 0x0340 */
	printf("HDD: root_dev: 0x%04x\n", root_dev);
	std::string tmp_str[n];
	CMenuWidget * tempMenu[n];
	for(int i = 0; i < n;i++) {
		tempMenu[i] = NULL;
		char str[256];
		char sstr[256];
		char vendor[128], model[128];
		int64_t bytes;
		int64_t megabytes;
		int removable = 0;
		bool isroot = false;

		printf("HDD: checking /sys/block/%s\n", namelist[i]->d_name);
		snprintf(str, sizeof(str), "/dev/%s", namelist[i]->d_name);
		fd = open(str, O_RDONLY);
		if(fd < 0) {
			printf("Cant open %s\n", str);
			continue;
		}
		if (ioctl(fd, BLKGETSIZE64, &bytes))
			perror("BLKGETSIZE64");

		ret = fstat(fd, &s);
		if (ret != -1) {
			if ((int)(s.st_rdev & 0x0ffc0) == root_dev) {
				isroot = true;
				/* dev_t is different sized on different architectures :-( */
				printf("-> root device is on this disk 0x%04x, skipping\n", (int)s.st_rdev);
			}
		}
		close(fd);

		megabytes = bytes/1000000;

		snprintf(str, sizeof(str), "/sys/block/%s/device/vendor", namelist[i]->d_name);
		f = fopen(str, "r");
		if(!f) {
			printf("Cant open %s\n", str);
			continue;
		}
		fscanf(f, "%s", vendor);
		fclose(f);

		snprintf(str, sizeof(str), "/sys/block/%s/device/model", namelist[i]->d_name);
		f = fopen(str, "r");
		if(!f) {
			printf("Cant open %s\n", str);
			continue;
		}
		fscanf(f, "%s", model);
		fclose(f);

		snprintf(str, sizeof(str), "/sys/block/%s/removable", namelist[i]->d_name);
		f = fopen(str, "r");
		if(!f) {
			printf("Cant open %s\n", str);
			continue;
		}
		fscanf(f, "%d", &removable);
		fclose(f);

		bool enabled = !CNeutrinoApp::getInstance()->recordingstatus && !removable && !isroot;

		snprintf(str, sizeof(str), "%s %s %ld %s", vendor, model, (long)(megabytes < 10000 ? megabytes : megabytes/1000), megabytes < 10000 ? "MB" : "GB");
		printf("HDD: %s\n", str);
		tmp_str[i]=str;
		tempMenu[i] = new CMenuWidget(str, NEUTRINO_ICON_SETTINGS);
		tempMenu[i]->addIntroItems();
		//tempMenu->addItem( new CMenuOptionChooser(LOCALE_HDD_FS, &g_settings.hdd_fs, HDD_FILESYS_OPTIONS, HDD_FILESYS_OPTION_COUNT, true));

		mf = new CMenuForwarder(LOCALE_HDD_FORMAT, true, "", &fmtexec, namelist[i]->d_name);
		mf->setHint("", LOCALE_MENU_HINT_HDD_FORMAT);
		tempMenu[i]->addItem(mf);

		mf = new CMenuForwarder(LOCALE_HDD_CHECK, true, "", &chkexec, namelist[i]->d_name);
		mf->setHint("", LOCALE_MENU_HINT_HDD_CHECK);
		tempMenu[i]->addItem(mf);

		snprintf(sstr, sizeof(sstr), "%s (%s)", g_Locale->getText(LOCALE_HDD_REMOVABLE_DEVICE),  namelist[i]->d_name);
		mf = new CMenuForwarderNonLocalized((removable ? sstr : namelist[i]->d_name), enabled, tmp_str[i], tempMenu[i]);
		mf->setHint("", LOCALE_MENU_HINT_HDD_TOOLS);
		hddmenu->addItem(mf);

		hdd_found = 1;
		free(namelist[i]);
	}
	if (n >= 0)
		free(namelist);

	if(!hdd_found)
		hddmenu->addItem(new CMenuForwarder(LOCALE_HDD_NOT_FOUND, false));

	ret = hddmenu->exec(NULL, "");
	for(int i = 0; i < n;i++) {
		if( hdd_found && tempMenu[i] != NULL ){
			delete tempMenu[i];
		}
	}

	delete hddmenu;
	return ret;
}

int CHDDDestExec::exec(CMenuTarget* /*parent*/, const std::string&)
{
	char M_opt[50],S_opt[50];
	char opt[100];
	struct dirent **namelist;
	int n = scandir("/sys/block", &namelist, my_filter, alphasort);

	if (n < 0)
		return 0;

	const char hdparm[] = "/sbin/hdparm";
	bool hdparm_link = false;
	struct stat stat_buf;
	if(::lstat(hdparm, &stat_buf) == 0)
		if( S_ISLNK(stat_buf.st_mode) )
			hdparm_link = true;

	for (int i = 0; i < n; i++) {
		printf("CHDDDestExec: noise %d sleep %d /dev/%s\n",
			 g_settings.hdd_noise, g_settings.hdd_sleep, namelist[i]->d_name);
		snprintf(S_opt, sizeof(S_opt),"-S%d", g_settings.hdd_sleep);
		snprintf(opt, sizeof(opt),"/dev/%s",namelist[i]->d_name);

		if(hdparm_link){
			//hdparm -M is not included in busybox hdparm!
			my_system(3, hdparm, S_opt, opt);
		}else{
			snprintf(M_opt, sizeof(M_opt),"-M%d", g_settings.hdd_noise);
			my_system(4, hdparm, M_opt, S_opt, opt);
		}
		free(namelist[i]);
	}
	free(namelist);
	return 1;
}

static int check_and_umount(char * dev, char * path)
{
	char buffer[255];
	FILE *f = fopen("/proc/mounts", "r");
	if(f == NULL)
		return -1;
	while (fgets (buffer, 255, f) != NULL) {
		if(strstr(buffer, dev)) {
			printf("HDD: %s mounted\n", path);
			fclose(f);
			return umount(path);
		}
	}
	fclose(f);
	printf("HDD: %s not mounted\n", path);
	return 0;
}

int CHDDFmtExec::exec(CMenuTarget* /*parent*/, const std::string& key)
{
	char cmd[100];
	char cmd2[100];
	CHintBox * hintbox;
	int res;
	FILE * f;
	char src[128], dst[128];
	CProgressWindow * progress;

	snprintf(src, sizeof(src), "/dev/%s1", key.c_str());
	snprintf(dst, sizeof(dst), "/media/%s1", key.c_str());

	printf("CHDDFmtExec: key %s\n", key.c_str());

	res = ShowMsgUTF ( LOCALE_HDD_FORMAT, g_Locale->getText(LOCALE_HDD_FORMAT_WARN), CMessageBox::mbrNo, CMessageBox::mbYes | CMessageBox::mbNo );
	if(res != CMessageBox::mbrYes)
		return 0;

	bool srun = my_system(3, "killall", "-9", "smbd");

	//res = check_and_umount(dst);
	res = check_and_umount(src, dst);
	printf("CHDDFmtExec: umount res %d\n", res);

	if(res) {
		hintbox = new CHintBox(LOCALE_HDD_FORMAT, g_Locale->getText(LOCALE_HDD_UMOUNT_WARN));
		hintbox->paint();
		sleep(2);
		delete hintbox;
		goto _return;
	}

	f = fopen("/proc/sys/kernel/hotplug", "w");
	if(f) {
		fprintf(f, "none\n");
		fclose(f);
	}

	progress = new CProgressWindow();
	progress->setTitle(LOCALE_HDD_FORMAT);
	progress->exec(NULL,"");
	progress->showStatusMessageUTF("Executing fdisk");
	progress->showGlobalStatus(0);

	if (access("/sbin/sfdisk", X_OK) == 0) {
		snprintf(cmd, sizeof(cmd), "/sbin/sfdisk -f -uM /dev/%s", key.c_str());
		strcpy(cmd2, "0,\n;\n;\n;\ny\n");
	} else {
		snprintf(cmd, sizeof(cmd), "/sbin/fdisk /dev/%s", key.c_str());
		strcpy(cmd2, "o\nn\np\n1\n\n\nw\n");
	}

	printf("CHDDFmtExec: executing %s\n", cmd);
	f=popen(cmd, "w");
	if (!f) {
		hintbox = new CHintBox(LOCALE_HDD_FORMAT, g_Locale->getText(LOCALE_HDD_FORMAT_FAILED));
		hintbox->paint();
		sleep(2);
		delete hintbox;
		goto _remount;
	}

	fprintf(f, "%s", cmd2);
	pclose(f);
	//sleep(1);

	switch(g_settings.hdd_fs) {
		case 0:
			snprintf(cmd, sizeof(cmd), "/sbin/mkfs.ext3 -T largefile -m0 %s", src);
			break;
		case 1:
			snprintf(cmd, sizeof(cmd), "/sbin/mkreiserfs -f -f %s", src);
			break;
		default:
			return 0;
	}

	printf("CHDDFmtExec: executing %s\n", cmd);

	f=popen(cmd, "r");
	if (!f) {
		hintbox = new CHintBox(LOCALE_HDD_FORMAT, g_Locale->getText(LOCALE_HDD_FORMAT_FAILED));
		hintbox->paint();
		sleep(2);
		delete hintbox;
		goto _remount;
	}

	char buf[256];
	setbuf(f, NULL);
	int n, t, in, pos, stage;
	pos = 0;
	stage = 0;
	while (true)
	{
		in = fgetc(f);
		if (in == EOF)
			break;

		buf[pos] = (char)in;
		pos++;
		buf[pos] = 0;
		if (in == '\b' || in == '\n')
			pos = 0; /* start a new line */
		//printf("%s", buf);
		switch (stage) {
			case 0:
				if (strcmp(buf, "Writing inode tables:") == 0) {
					stage++;
					progress->showGlobalStatus(20);
					progress->showStatusMessageUTF(buf);
				}
				break;
			case 1:
				if (in == '\b' && sscanf(buf, "%d/%d\b", &n, &t) == 2) {
					int percent = 100 * n / t;
					progress->showLocalStatus(percent);
					progress->showGlobalStatus(20 + percent / 5);
				}
				if (strstr(buf, "done")) {
					stage++;
					pos = 0;
				}
				break;
			case 2:
				if (strstr(buf, "blocks):") && sscanf(buf, "Creating journal (%d blocks):", &n) == 1) {
					progress->showLocalStatus(0);
					progress->showGlobalStatus(60);
					progress->showStatusMessageUTF(buf);
					pos = 0;
				}
				if (strstr(buf, "done")) {
					stage++;
					pos = 0;
				}
				break;
			case 3:
				if (strcmp(buf, "Writing superblocks and filesystem accounting information:") == 0) {
					progress->showGlobalStatus(80);
					progress->showStatusMessageUTF(buf);
					pos = 0;
				}
				break;
			default:
				// printf("unknown stage! %d \n\t", stage);
				break;
		}
	}
	progress->showLocalStatus(100);
	pclose(f);
	progress->showGlobalStatus(100);
	sleep(2);

	printf("CHDDFmtExec: executing %s %s\n","/sbin/tune2fs -r 0 -c 0 -i 0", src);
	my_system(8, "/sbin/tune2fs", "-r", "0", "-c", "0", "-i", "0", src);

_remount:
	progress->hide();
	delete progress;

        switch(g_settings.hdd_fs) {
                case 0:
			safe_mkdir(dst);
			res = mount(src, dst, "ext3", 0, NULL);
                        break;
                case 1:
			safe_mkdir(dst);
			res = mount(src, dst, "reiserfs", 0, NULL);
                        break;
		default:
                        break;
        }
	f = fopen("/proc/sys/kernel/hotplug", "w");
	if(f) {
		fprintf(f, "/sbin/hotplug\n");
		fclose(f);
	}

	if(!res) {
		snprintf(cmd, sizeof(cmd), "%s/movies", dst);
		safe_mkdir(cmd);
		snprintf(cmd, sizeof(cmd), "%s/pictures", dst);
		safe_mkdir(cmd);
		snprintf(cmd, sizeof(cmd), "%s/epg", dst);
		safe_mkdir(cmd);
		snprintf(cmd, sizeof(cmd), "%s/music", dst);
		safe_mkdir(cmd);
		snprintf(cmd, sizeof(cmd), "%s/logos", dst);
		safe_mkdir(cmd);
		snprintf(cmd, sizeof(cmd), "%s/plugins", dst);
		safe_mkdir(cmd);
		sync();
	}
_return:
	if (!srun) my_system(1, "smbd");
	return menu_return::RETURN_REPAINT;
}

int CHDDChkExec::exec(CMenuTarget* /*parent*/, const std::string& key)
{
	char cmd[100];
	CHintBox * hintbox;
	int res;
	char src[128], dst[128];
	FILE * f;
	CProgressWindow * progress;
	int oldpass = 0, pass, step, total;
	int percent = 0, opercent = 0;

	snprintf(src, sizeof(src), "/dev/%s1", key.c_str());
	snprintf(dst, sizeof(dst), "/media/%s1", key.c_str());

printf("CHDDChkExec: key %s\n", key.c_str());

	bool srun = my_system(3, "killall", "-9", "smbd");

	//res = check_and_umount(dst);
	res = check_and_umount(src, dst);
	printf("CHDDChkExec: umount res %d\n", res);
	if(res) {
		hintbox = new CHintBox(LOCALE_HDD_CHECK, g_Locale->getText(LOCALE_HDD_UMOUNT_WARN));
		hintbox->paint();
		sleep(2);
		delete hintbox;
		return menu_return::RETURN_REPAINT;
	}

	switch(g_settings.hdd_fs) {
		case 0:
			snprintf(cmd, sizeof(cmd), "/sbin/fsck.ext3 -C 1 -f -y %s", src);
			break;
		case 1:
			snprintf(cmd, sizeof(cmd), "/sbin/reiserfsck --fix-fixable %s", src);
			break;
		default:
			return 0;
	}

	printf("CHDDChkExec: Executing %s\n", cmd);
	f=popen(cmd, "r");
	if(!f) {
		hintbox = new CHintBox(LOCALE_HDD_CHECK, g_Locale->getText(LOCALE_HDD_CHECK_FAILED));
		hintbox->paint();
		sleep(2);
		delete hintbox;
		goto ret1;
	}

	progress = new CProgressWindow();
	progress->setTitle(LOCALE_HDD_CHECK);
	progress->exec(NULL,"");

	char buf[256];
	while(fgets(buf, 255, f) != NULL)
	{
		if(isdigit(buf[0])) {
			sscanf(buf, "%d %d %d\n", &pass, &step, &total);
			if(total == 0)
				total = 1;
			if(oldpass != pass) {
				oldpass = pass;
				progress->showGlobalStatus(pass > 0 ? (pass-1)*20: 0);
			}
			percent = (step * 100) / total;
			if(opercent != percent) {
				opercent = percent;
//printf("CHDDChkExec: pass %d : %d\n", pass, percent);
				progress->showLocalStatus(percent);
			}
		}
		else if(!strncmp(buf, "Pass", 4)) {
			char *t = strrchr(buf, '\n');
			if (t)
				*t = 0;
			progress->showStatusMessageUTF(buf);
		}
	}
//printf("CHDDChkExec: %s\n", buf);
	pclose(f);
	progress->showGlobalStatus(100);
	progress->showStatusMessageUTF(buf);
	sleep(2);
	progress->hide();
	delete progress;

ret1:
        switch(g_settings.hdd_fs) {
                case 0:
			safe_mkdir(dst);
			res = mount(src, dst, "ext3", 0, NULL);
                        break;
                case 1:
			safe_mkdir(dst);
			res = mount(src, dst, "reiserfs", 0, NULL);
                        break;
		default:
                        break;
        }
	printf("CHDDChkExec: mount res %d\n", res);

	if (!srun) my_system(1, "smbd");
	return menu_return::RETURN_REPAINT;
}
