/*
	lcd4l

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

#ifndef __lcd4l__
#define __lcd4l__

//#define BP
#define NI

#include <string>

class CLCD4l
{
	public:
		CLCD4l();
		~CLCD4l();

		// Functions
		void	InitLCD4l();
		void	StartLCD4l();
		void	StopLCD4l();
		void	SwitchLCD4l();

		int	CreateFile(const char *file, std::string content = "", bool convert = false);
		int	RemoveFile(const char *file);

	private:
		enum 
		{
			MODE_UNKNOWN = -1,
			MODE_TV = 1,
			MODE_RADIO = 2,
			MODE_SCART = 3,
			MODE_STANDBY = 4,
			MODE_AUDIO = 5,
			MODE_PIC = 6,
			MODE_TS = 7,
			MODE_OFF = 8,
			MODE_MASK = 0xFF,
			NOREZAP = 0x100
		};

		pthread_t	thrLCD4l;
		static void*	LCD4lProc(void *arg);

		struct tm	*tm_struct;

		// Functions
		void		Init();
		void		ParseInfo(uint64_t parseID, bool newID, bool firstRun = false);

		uint64_t	GetParseID();
		bool		CompareParseID(uint64_t &i_ParseID);
		bool		GetLogoName(uint64_t channel_id, std::string channel_name, std::string & logo);

		std::string	hexStr(unsigned char* data);
		void		strReplace(std::string & orig, const char *fstr, const std::string rstr);
		bool		WriteFile(const char *file, std::string content = "", bool convert = false);

		// Variables
		uint64_t	m_ParseID;
		int		m_Mode;
		int		m_ModeChannel;

		int		m_Tuner;
		int		m_Volume;
		int		m_ModeRec;
		int		m_ModeTshift;
		int		m_ModeTimer;
		int		m_ModeEcm;
#ifdef BP
		int		m_ModeNews;
#endif

		std::string	m_Service;
		int		m_ChannelNr;
		std::string	m_Logo;
		int		m_ModeLogo;

		std::string	m_Layout;

		std::string	m_Event;
		int		m_Progress;
		char		m_Duration[15];
		std::string	m_Start;
		std::string	m_End;

		std::string	m_font;
		std::string	m_fgcolor;
		std::string	m_bgcolor;
};

#endif
