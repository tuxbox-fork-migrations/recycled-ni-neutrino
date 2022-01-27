/*
	lcd4l setup

	Copyright (C) 2012 'defans'
	Homepage: http://www.bluepeercrew.us/

	Copyright (C) 2012-2021 'vanhofen'
	Homepage: http://www.neutrino-images.de/

	Copyright (C) 2016-2018 'TangoCash'
		  (C) 2021, Thilo Graf 'dbt'

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
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef __lcd4l_setup__
#define __lcd4l_setup__

#include "gui/widget/menue.h"
#include "gui/widget/hintbox.h"
#include <sigc++/signal.h>

class CLCD4lSetup : public CMenuTarget, CChangeObserver
{
	private:
		bool lcd4l_display_type_changed;
		int temp_lcd4l_display_type;
		int temp_lcd4l_skin;
		int temp_lcd4l_brightness;
		int temp_lcd4l_screenshots;

		int width;
		int show();
		int showTypeSetup();

		//messages
		CHint *hint;
		void removeHint();
		void showHint(const std::string &text);
		//slots
		sigc::slot<void> sl_start, sl_stop, sl_restart, sl_remove;

	public:
		static CLCD4lSetup* getInstance();
		CLCD4lSetup();
		~CLCD4lSetup();
		int exec(CMenuTarget *parent, const std::string &actionkey);
		virtual bool changeNotify(const neutrino_locale_t OptionName, void * /*data*/);
		void connectSlots();
};

#endif
