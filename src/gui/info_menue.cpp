/*
	info menue - Neutrino-GUI

	Copyright (C) 2001 Steffen Hehn 'McClean'
	and some other guys
	Homepage: http://dbox.cyberphoria.org/

	Copyright (C) 2011 T. Graf 'dbt'
	Homepage: http://www.dbox2-tuning.net/

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

#include <global.h>
#include <neutrino.h>
#include <neutrino_menue.h>

#include <cs_api.h> //NI
#include <gui/hdd_info.h> //NI
#include <gui/info_menue.h>
#include <gui/imageinfo_ni.h> //NI
#include <gui/dboxinfo.h>
#include <gui/streaminfo2.h>

#if 0
#include <gui/buildinfo.h>
#endif
#include <gui/widget/messagebox.h> //NI

#include <driver/screen_max.h>
#include "gui/cam_menu.h"

extern CCAMMenuHandler * g_CamHandler;

CInfoMenu::CInfoMenu()
{
	width = 35; //NI
}

CInfoMenu::~CInfoMenu()
{
}

int CInfoMenu::exec(CMenuTarget* parent, const std::string &actionKey) //NI
{
	int   res = menu_return::RETURN_REPAINT;

	if (parent != NULL)
		parent->hide();

	//NI
	if (actionKey == "info")
	{
		char str[1024];
		sprintf(str, "cs_get_revision(): 0x%02X\n", cs_get_revision());
#ifdef BOXMODEL_APOLLO
		sprintf(str, "%scs_get_chip_type(): 0x%04X\n", str, cs_get_chip_type());
#endif
		ShowMsg(LOCALE_MESSAGEBOX_INFO, str, CMessageBox::mbrBack, CMessageBox::mbBack);
		return res;
	}

	res = showMenu();

	return res;
}

int CInfoMenu::showMenu()
{
	CMenuWidget *info = new CMenuWidget(LOCALE_MESSAGEBOX_INFO, NEUTRINO_ICON_INFO, width, MN_WIDGET_ID_INFOMENUE);
	info->addKey(CRCInput::RC_info, this, "info"); //NI

	CImageInfoNI imageinfo; //NI
	CDBoxInfoWidget boxinfo;
	CHDDInfoMenu hddinfo; //NI
	CStreamInfo2 streaminfo;

	info->addIntroItems();
	CMenuForwarder * mf = new CMenuForwarder(LOCALE_SERVICEMENU_IMAGEINFO,  true, NULL, &imageinfo, NULL, CRCInput::RC_red);
	mf->setHint(NEUTRINO_ICON_HINT_IMAGEINFO, LOCALE_MENU_HINT_IMAGEINFO);
	info->addItem(mf);

	mf = new CMenuForwarder(LOCALE_EXTRA_DBOXINFO,         true, NULL, &boxinfo, NULL, CRCInput::RC_green);
	mf->setHint(NEUTRINO_ICON_HINT_DBOXINFO, LOCALE_MENU_HINT_DBOXINFO);
	info->addItem(mf);

	mf = new CMenuForwarder(LOCALE_STREAMINFO_HEAD,        !CNeutrinoApp::getInstance()->channelList->isEmpty(), NULL, &streaminfo, NULL, CRCInput::RC_yellow);
	mf->setHint(NEUTRINO_ICON_HINT_STREAMINFO, LOCALE_MENU_HINT_STREAMINFO);
	info->addItem(mf);
	
#if 0
	CBuildInfo buildinfo;
	mf = new CMenuForwarder(LOCALE_BUILDINFO_MENU,  true, NULL, &buildinfo, NULL, CRCInput::RC_blue);
	mf->setHint(NEUTRINO_ICON_HINT_IMAGEINFO, LOCALE_MENU_HINT_BUILDINFO);
	info->addItem(mf);
#endif

	if (g_settings.easymenu) { //NI todo (remove blue button, we use it!)
		mf = new CMenuForwarder(LOCALE_CI_SETTINGS, true, NULL, g_CamHandler,  NULL, CRCInput::RC_blue);
		mf->setHint(NEUTRINO_ICON_HINT_CI, LOCALE_MENU_HINT_CI);
		info->addItem(mf);
	}

	//NI
	mf = new CMenuForwarder(LOCALE_HDD_INFO_HEAD, true, NULL, &hddinfo, NULL, CRCInput::RC_blue);
	mf->setHint(NEUTRINO_ICON_HINT_HDD_INFO, LOCALE_MENU_HINT_HDD_INFO);
	info->addItem(mf);

	//add I_TYPE_INFORMATION plugins
	info->integratePlugins(CPlugins::I_TYPE_INFORMATION, 1);

	int res = info->exec(NULL, "");
	delete info;
	return res;
}
