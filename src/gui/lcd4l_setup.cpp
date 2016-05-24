/*
	lcd4l setup

	Copyright (C) 2012 'defans'
	Homepage: http://www.bluepeercrew.us/

	Copyright (C) 2012-2016 'vanhofen'
	Homepage: http://www.neutrino-images.de/

	Modded    (C) 2016 'TangoCash'

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
#include <iostream>
#include <fstream>
#include <sstream>
#include <signal.h>
#include <unistd.h>

#include <global.h>
#include <neutrino.h>
#include <mymenu.h>
#include <neutrino_menue.h>

#include <gui/filebrowser.h>
#include <gui/widget/hintbox.h>
#include <gui/widget/icons.h>
#include <gui/widget/menue.h>

#include <gui/lcd4l_setup.h>

#include <system/helpers.h>

#include <driver/screen_max.h>

// lcd4l-support
#include "gui/lcd4l.h"
extern CLCD4l *LCD4l;

#define FLAG_DIR "/var/etc/"

const CMenuOptionChooser::keyval LCD4L_SUPPORT_OPTIONS[] =
{
	{ 0, LOCALE_LCD4L_SUPPORT_OFF },
	{ 1, LOCALE_LCD4L_SUPPORT_AUTO },
	{ 2, LOCALE_LCD4L_SUPPORT_ON }
};
#define LCD4L_SUPPORT_OPTION_COUNT (sizeof(LCD4L_SUPPORT_OPTIONS)/sizeof(CMenuOptionChooser::keyval))

const CMenuOptionChooser::keyval LCD4L_SKIN_OPTIONS[] =
{
	{ 0, LOCALE_LCD4L_SKIN_0 },
	{ 1, LOCALE_LCD4L_SKIN_1 },
	{ 2, LOCALE_LCD4L_SKIN_2 },
	{ 3, LOCALE_LCD4L_SKIN_3 },
	{ 4, LOCALE_LCD4L_SKIN_4 }
};
#define LCD4L_SKIN_OPTION_COUNT (sizeof(LCD4L_SKIN_OPTIONS)/sizeof(CMenuOptionChooser::keyval))

CLCD4lSetup::CLCD4lSetup()
{
	width = 40;
}

CLCD4lSetup::~CLCD4lSetup()
{
}

int CLCD4lSetup::exec(CMenuTarget* parent, const std::string &actionkey)
{
	printf("CLCD4lSetup::exec: actionkey %s\n", actionkey.c_str());
	int res = menu_return::RETURN_REPAINT;

        if (parent)
                parent->hide();

	if (actionkey == "lcd4l_logodir") {
		const char *action_str = "lcd4l_logodir";
		chooserDir(g_settings.lcd4l_logodir, false, action_str);
		return menu_return::RETURN_REPAINT;
	}

	res = show();

	return res;
}

bool CLCD4lSetup::changeNotify(const neutrino_locale_t OptionName, void * /*data*/)
{
#if 0
	int value = 0;

	if (data)
		value = (*(int *)data);
#endif

	if (ARE_LOCALES_EQUAL(OptionName, LOCALE_LCD4L_SUPPORT))
	{
		LCD4l->StopLCD4l();
		if (g_settings.lcd4l_support)
			LCD4l->StartLCD4l();
	}

	return false;
}

int CLCD4lSetup::show()
{
	int shortcut = 1;

	int temp_lcd4l_skin	= g_settings.lcd4l_skin;

	// lcd4l setup
	CMenuWidget* lcd4lSetup = new CMenuWidget(LOCALE_LCD4L_SUPPORT, NEUTRINO_ICON_SETTINGS, width, MN_WIDGET_ID_LCD4L_SETUP);
	lcd4lSetup->addIntroItems();

	mc = new CMenuOptionChooser(LOCALE_LCD4L_SUPPORT, &g_settings.lcd4l_support, LCD4L_SUPPORT_OPTIONS, LCD4L_SUPPORT_OPTION_COUNT, true, this, CRCInput::RC_red);
	mc->setHint("", LOCALE_MENU_HINT_LCD4L_SUPPORT);
	lcd4lSetup->addItem(mc);
	lcd4lSetup->addItem(GenericMenuSeparatorLine);

	mf = new CMenuForwarder(LOCALE_LCD4L_LOGODIR, true, g_settings.lcd4l_logodir, this, "lcd4l_logodir", CRCInput::convertDigitToKey(shortcut++));
	mf->setHint("", LOCALE_MENU_HINT_LCD4L_LOGODIR);
	lcd4lSetup->addItem(mf);

	mc = new CMenuOptionChooser(LOCALE_LCD4L_SKIN, &temp_lcd4l_skin, LCD4L_SKIN_OPTIONS, LCD4L_SKIN_OPTION_COUNT, true, NULL, CRCInput::convertDigitToKey(shortcut++));
	mc->setHint("", LOCALE_MENU_HINT_LCD4L_SKIN);
	lcd4lSetup->addItem(mc);

	const char *flag_lcd4l_weather = FLAG_DIR ".lcd-weather";
	int fake_lcd4l_weather = file_exists(flag_lcd4l_weather);
	CTouchFileNotifier * lcd_weather = new CTouchFileNotifier(flag_lcd4l_weather);
	mc = new CMenuOptionChooser(LOCALE_LCD4L_WEATHER, &fake_lcd4l_weather, OPTIONS_OFF0_ON1_OPTIONS, OPTIONS_OFF0_ON1_OPTION_COUNT, (file_exists("/share/lcd/scripts/weather")), lcd_weather, CRCInput::convertDigitToKey(shortcut++));
	mc->setHint("", LOCALE_MENU_HINT_LCD4L_WEATHER);
	lcd4lSetup->addItem(mc);

	const char *flag_lcd4l_clock_a = FLAG_DIR ".lcd-clock_a";
	int fake_lcd4l_clock_a = file_exists(flag_lcd4l_clock_a);
	CTouchFileNotifier * lcd_clock_a = new CTouchFileNotifier(flag_lcd4l_clock_a);
	mc = new CMenuOptionChooser(LOCALE_LCD4L_CLOCK_A, &fake_lcd4l_clock_a, OPTIONS_OFF0_ON1_OPTIONS, OPTIONS_OFF0_ON1_OPTION_COUNT, true, lcd_clock_a, CRCInput::convertDigitToKey(shortcut++));
	mc->setHint("", LOCALE_MENU_HINT_LCD4L_CLOCK_A);
	lcd4lSetup->addItem(mc);

	lcd4lSetup->addItem(GenericMenuSeparator);

	mc = new CMenuOptionChooser(LOCALE_LCD4L_CONVERT, &g_settings.lcd4l_convert, OPTIONS_OFF0_ON1_OPTIONS, OPTIONS_OFF0_ON1_OPTION_COUNT, true, NULL, CRCInput::convertDigitToKey(shortcut++));
	mc->setHint(NEUTRINO_ICON_HINT_LCD4L, LOCALE_MENU_HINT_LCD4L_CONVERT);
	lcd4lSetup->addItem(mc);

	int res = lcd4lSetup->exec(NULL, "");

	lcd4lSetup->hide();
	delete lcd4lSetup;

	// the things to do on exit

	if (g_settings.lcd4l_skin != temp_lcd4l_skin)
	{
		g_settings.lcd4l_skin = temp_lcd4l_skin;
		LCD4l->InitLCD4l();
	}

	return res;
}
