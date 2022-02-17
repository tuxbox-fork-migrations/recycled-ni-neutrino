/*
	lcd4l

	Copyright (C) 2012 'defans'
	Homepage: http://www.bluepeercrew.us/

	Copyright (C) 2012-2018 'vanhofen'
	Homepage: http://www.neutrino-images.de/

	Copyright (C) 2016-2019 'TangoCash'
	Copyright (C) 2021, Thilo Graf 'dbt'

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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif


#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <iomanip>
#include <system/set_threadname.h>

#include <global.h>
#include <neutrino.h>

#include <timerdclient/timerdclient.h>
#include <system/helpers.h>
#include <system/debug.h>
#include <driver/record.h>
#include <driver/audioplay.h>
#include <driver/radiotext.h>
#include <zapit/capmt.h>
#include <zapit/zapit.h>
#include <gui/infoviewer.h>
#include <gui/movieplayer.h>
#include <gui/pictureviewer.h>
#include <eitd/sectionsd.h>
#include <hardware/video.h>
#include <gui/weather.h>

#include "lcd4l.h"

extern CRemoteControl *g_RemoteControl;
extern cVideo *videoDecoder;
extern CPictureViewer *g_PicViewer;

#define LCD_DATADIR		"/tmp/lcd/"

#define LCD_ICONSDIR		TARGET_PREFIX "/share/lcd/icons/"
#define ICONSEXT		".png"

#define LOGO_DUMMY		LCD_ICONSDIR "blank.png"

#define BRIGHTNESS		LCD_DATADIR "brightness"
#define BRIGHTNESS_STANDBY	LCD_DATADIR "brightness_standby"
#define RESOLUTION		LCD_DATADIR "resolution"
#define ASPECTRATIO		LCD_DATADIR "aspectratio"
#define VIDEOTEXT		LCD_DATADIR "videotext"
#define RADIOTEXT		LCD_DATADIR "radiotext"
#define DOLBYDIGITAL		LCD_DATADIR "dolbydigital"
#define TUNER			LCD_DATADIR "tuner"
#define TUNER_SIG		LCD_DATADIR "tuner_sig"
#define TUNER_SNR		LCD_DATADIR "tuner_snr"
#define TUNER_BER		LCD_DATADIR "tuner_ber"
#define VOLUME			LCD_DATADIR "volume"
#define MODE_REC		LCD_DATADIR "mode_rec"
#define MODE_TSHIFT		LCD_DATADIR "mode_tshift"
#define MODE_TIMER		LCD_DATADIR "mode_timer"
#define MODE_ECM		LCD_DATADIR "mode_ecm"
#define MODE_CAM		LCD_DATADIR "mode_cam"

#define SERVICE			LCD_DATADIR "service"
#define CHANNELNR		LCD_DATADIR "channelnr"
#define LOGO			LCD_DATADIR "logo"
#define MODE_LOGO		LCD_DATADIR "mode_logo"
#define LAYOUT			LCD_DATADIR "layout"

#define EVENT			LCD_DATADIR "event"
#define INFO1			LCD_DATADIR "info1"
#define INFO2			LCD_DATADIR "info2"
#define PROGRESS		LCD_DATADIR "progress"
#define DURATION		LCD_DATADIR "duration"
#define START			LCD_DATADIR "start"
#define END			LCD_DATADIR "end"

#define MENU			LCD_DATADIR "menu"

#define FONT			LCD_DATADIR "font"
#define FGCOLOR			LCD_DATADIR "fgcolor"
#define BGCOLOR			LCD_DATADIR "bgcolor"

#define FCOLOR1			LCD_DATADIR "fcolor1"
#define FCOLOR2			LCD_DATADIR "fcolor2"
#define PBCOLOR			LCD_DATADIR "pbcolor"

#define WEATHER_CITY		LCD_DATADIR "weather_city"
#define WEATHER_TIMESTAMP	LCD_DATADIR "weather_timestamp"
#define WEATHER_TEMP		LCD_DATADIR "weather_temp"
#define WEATHER_WIND		LCD_DATADIR "weather_wind"
#define WEATHER_ICON		LCD_DATADIR "weather_icon"

#define FLAG_LCD4LINUX		"/tmp/.lcd4linux"
#define PIDFILE			"/var/run/lcd4linux.pid"

/* ----------------------------------------------------------------- */

CLCD4l::CLCD4l()
{
	thrLCD4l = NULL;
	exit_proc = false;
}

CLCD4l::~CLCD4l()
{
	exit_proc = true;
	if (thrLCD4l)
		thrLCD4l->join();
	delete thrLCD4l;
	thrLCD4l = NULL;
}

CLCD4l *CLCD4l::getInstance()
{
	static CLCD4l *me = NULL;
	if (!me)
		me = new CLCD4l();
	return me;
}

/* ----------------------------------------------------------------- */

void CLCD4l::InitLCD4l()
{
	if (thrLCD4l)
	{
		dprintf(DEBUG_NORMAL, "\033[32m[CLCD4l] [%s - %d] initializing CLCD4l \033[0m\n", __func__, __LINE__);
		Init();
	}
}

void CLCD4l::StartLCD4l()
{
	OnBeforeStart();
	if (!thrLCD4l)
	{
		dprintf(DEBUG_NORMAL, "\033[32m[CLCD4l] [%s - %d] starting thread with mode %d \033[0m\n", __func__, __LINE__, g_settings.lcd4l_support);

		exit_proc = false;
		thrLCD4l = new std::thread(LCD4lProc, this);
		dprintf(DEBUG_NORMAL, "\033[32m[CLCD4l] [%s - %d] thread [%p] is running\033[0m\n", __func__, __LINE__, thrLCD4l);
	}

	if (g_settings.lcd4l_support)
	{
		if (exec_initscript("lcd4linux", "start"))
			OnAfterStart();
		else
			OnError();
	}
}

void CLCD4l::StopLCD4l()
{
	OnBeforeStop();
	if (thrLCD4l)
	{
		dprintf(DEBUG_NORMAL, "\033[32m[CLCD4l] [%s - %d] stopping thread [%p]\033[0m\n", __func__, __LINE__, thrLCD4l);

		exit_proc = true;
		thrLCD4l->join();
		dprintf(DEBUG_NORMAL, "\033[32m[CLCD4l] [%s - %d] thread [%p] joined\033[0m\n", __func__, __LINE__, thrLCD4l);

		delete thrLCD4l;
		thrLCD4l = NULL;
		dprintf(DEBUG_NORMAL, "\033[32m[CLCD4l] [%s - %d] thread [%p] terminated\033[0m\n", __func__, __LINE__, thrLCD4l);
	}

	if (exec_initscript("lcd4linux", "stop"))
		OnAfterStop();
	else
		OnError();
}

void CLCD4l::SwitchLCD4l()
{
	if (thrLCD4l)
		StopLCD4l();
	else
		StartLCD4l();
}

int CLCD4l::CreateFile(const char *file, std::string content, bool convert)
{
	// returns 0 = ok; 1 = can't create file; -1 = thread not found

	int ret = 0;

	if (thrLCD4l)
	{
		if (WriteFile(file, content, convert) == false)
			ret = 1;
	}
	else
		ret = -1;

	return ret;
}

int CLCD4l::RemoveFile(const char *file)
{
	// returns 0 = ok; 1 = can't remove file;

	int ret = 0;

	if (access(file, F_OK) == 0)
	{
		if (unlink(file) != 0)
			ret = 1;
	}

	return ret;
}

int CLCD4l::CreateMenuFile(std::string content, bool convert)
{
	return CreateFile(MENU, content, convert);
}

int CLCD4l::RemoveMenuFile()
{
	return RemoveFile(MENU);
}

int CLCD4l::GetMaxBrightness()
{
	int max_brightness;

	switch (g_settings.lcd4l_display_type)
	{
		case SPF800x480:
		case SPF800x600:
		case SPF1024x600:
			max_brightness = 10;
			break;
		case DPF320x240:
		default:
			max_brightness = 7;
			break;
	}

	return max_brightness;
}

/* ----------------------------------------------------------------- */

void CLCD4l::Init()
{
	m_ParseID	= 0;

	m_Brightness	= -1;
	m_Brightness_standby = -1;
	m_Resolution	= "n/a";
	m_AspectRatio	= "n/a";
	m_Videotext	= -1;
	m_Radiotext	= -1;
	m_DolbyDigital	= "n/a";
	m_Tuner		= -1;
	m_Tuner_sig	= -1;
	m_Tuner_snr	= -1;
	m_Tuner_ber	= -1;
	m_Volume	= -1;
	m_ModeRec	= -1;
	m_RecordCount	= -1;
	m_ModeTshift	= -1;
	m_ModeTimer	= -1;
	m_ModeEcm	= -1;
	m_ModeCamPresent = false;
	m_ModeCam	= -1;

	m_Service	= "n/a";
	m_ChannelNr	= -1;
	m_Logo		= "n/a";
	m_ModeLogo	= -1;

	m_Layout	= "n/a";

	m_Event		= "n/a";
	m_Info1		= "n/a";
	m_Info2		= "n/a";
	m_Progress	= -1;
	for (int i = 0; i < (int)sizeof(m_Duration); i++)
		m_Duration[i] = ' ';
	m_Start		= "00:00";
	m_End		= "00:00";

	m_wcity		= "";
	m_wtimestamp	= "";
	m_wtemp		= "";
	m_wwind		= "";
	m_wicon		= "";

	if (!access(LCD_DATADIR, F_OK) == 0)
		mkdir(LCD_DATADIR, 0755);

	wait4daemon = true;
}

void *CLCD4l::LCD4lProc(void *arg)
{
	CLCD4l *PLCD4l = static_cast<CLCD4l *>(arg);
	set_threadname("lcd4l");
	PLCD4l->Init();

	sleep(5); //please wait !

	static bool FirstRun = true;
	uint64_t p_ParseID = 0;
	bool NewParseID = false;

	//printf("[CLCD4l] %s: starting loop\n", __FUNCTION__);
	while (!PLCD4l->exit_proc)
	{
		if ((!access(PIDFILE, F_OK) == 0) && (!FirstRun))
		{
			if (g_settings.lcd4l_support == 1) // automatic
			{
				if (PLCD4l->GetWaitStatus())
				{
					//printf("[CLCD4l] %s: waiting for lcd4linux\n", __FUNCTION__);
					sleep(10);
					continue;
				}
				else
					PLCD4l->SetWaitStatus(true);
			}
		}

		for (int i = 0; i < 10; i++)
		{
			usleep(5 * 100 * 1000); // 0.5 sec
			NewParseID = PLCD4l->CompareParseID(p_ParseID);
			if (NewParseID || p_ParseID == NeutrinoModes::mode_audio)
				break;
		}

		//printf("[CLCD4l] %s: m_ParseID: %llx (NewParseID: %d)\n", __FUNCTION__, p_ParseID, NewParseID ? 1 : 0);
		PLCD4l->ParseInfo(p_ParseID, NewParseID, FirstRun);

		if (FirstRun)
		{
			PLCD4l->WriteFile(FLAG_LCD4LINUX);
			FirstRun = false;
		}
	}
	return 0;
}

void CLCD4l::ParseInfo(uint64_t parseID, bool newID, bool firstRun)
{
	SNeutrinoTheme &t = g_settings.theme;

	std::string font = g_settings.font_file;
	font += "\n" + g_settings.font_file_monospace;

	if (m_font.compare(font))
	{
		WriteFile(FONT, font);
		m_font = font;
	}

	/* ----------------------------------------------------------------- */

	std::string fgcolor = hexStr(t.infobar_Text_red)
			+ hexStr(t.infobar_Text_green)
			+ hexStr(t.infobar_Text_blue)
			+ hexStrA2A(t.infobar_Text_alpha);

	if (m_fgcolor.compare(fgcolor))
	{
		WriteFile(FGCOLOR, fgcolor);
		m_fgcolor = fgcolor;
	}

	/* ----------------------------------------------------------------- */

	std::string bgcolor = hexStr(t.infobar_red)
			+ hexStr(t.infobar_green)
			+ hexStr(t.infobar_blue)
			+ hexStrA2A(t.infobar_alpha);

	if (m_bgcolor.compare(bgcolor))
	{
		WriteFile(BGCOLOR, bgcolor);
		m_bgcolor = bgcolor;
	}

	/* ----------------------------------------------------------------- */

	std::string fcolor1 = hexStr(t.infobar_Text_red)
			+ hexStr(t.infobar_Text_green)
			+ hexStr(t.infobar_Text_blue)
			+ hexStr(t.infobar_Text_alpha);

	if (m_fcolor1.compare(fcolor1))
	{
		WriteFile(FCOLOR1, fcolor1);
		m_fcolor1 = fcolor1;
	}

	/* ----------------------------------------------------------------- */

	std::string fcolor2 = hexStr(t.colored_events_red)
			+ hexStr(t.colored_events_green)
			+ hexStr(t.colored_events_blue)
			+ hexStr(t.colored_events_alpha);

	if (!t.colored_events_infobar)
		fcolor2 = fcolor1;

	if (m_fcolor2.compare(fcolor2))
	{
		WriteFile(FCOLOR2, fcolor2);
		m_fcolor2 = fcolor2;
	}

	/* ----------------------------------------------------------------- */

	std::string pbcolor = hexStr(t.menu_Content_Selected_red)
			+ hexStr(t.menu_Content_Selected_green)
			+ hexStr(t.menu_Content_Selected_blue)
			+ hexStrA2A(t.menu_Content_Selected_alpha);

	if (m_pbcolor.compare(pbcolor))
	{
		WriteFile(PBCOLOR, pbcolor);
		m_pbcolor = pbcolor;
	}

	/* ----------------------------------------------------------------- */

	int Brightness = g_settings.lcd4l_brightness;
	if (m_Brightness != Brightness)
	{
		WriteFile(BRIGHTNESS, to_string(Brightness));
		m_Brightness = Brightness;
	}

	int Brightness_standby = g_settings.lcd4l_brightness_standby;
	if (m_Brightness_standby != Brightness_standby)
	{
		WriteFile(BRIGHTNESS_STANDBY, to_string(Brightness_standby));
		m_Brightness_standby = Brightness_standby;
	}

	/* ----------------------------------------------------------------- */

	int x_res, y_res, framerate;
	if (videoDecoder)
	{
		// Hack: That should not happen, but while shutting down there
		// could be a null pointer and this can lead to a crash.
		// This behavior was observed with LeakSanitizer on pc hardware.
		videoDecoder->getPictureInfo(x_res, y_res, framerate);
	}
	else
		return;


	if (y_res == 1088)
		y_res = 1080;

	std::string Resolution = to_string(x_res) + "x" + to_string(y_res);
	//Resolution += "\n" + to_string(framerate); //TODO

	if (m_Resolution.compare(Resolution))
	{
		WriteFile(RESOLUTION, Resolution);
		m_Resolution = Resolution;
	}

	/* ----------------------------------------------------------------- */

	std::string AspectRatio;
	switch (videoDecoder->getAspectRatio())
	{
		case 0:
			AspectRatio = "n/a";
			break;
		case 1:
			AspectRatio = "4:3";
			break;
		case 2:
			AspectRatio = "14:9";
			break;
		case 3:
			AspectRatio = "16:9";
			break;
		case 4:
			AspectRatio = "20:9";
			break;
		default:
			AspectRatio = "n/k";
			break;
	}

	if (m_AspectRatio.compare(AspectRatio))
	{
		WriteFile(ASPECTRATIO, AspectRatio);
		m_AspectRatio = AspectRatio;
	}

	/* ----------------------------------------------------------------- */

	int Videotext = g_RemoteControl->current_PIDs.PIDs.vtxtpid;

	if (m_Videotext != Videotext)
	{
		WriteFile(VIDEOTEXT, Videotext ? "yes" : "no");
		m_Videotext = Videotext;
	}

	/* ----------------------------------------------------------------- */

	int Radiotext = 0;
	if (m_Mode == NeutrinoModes::mode_radio && g_settings.radiotext_enable && g_Radiotext)
		Radiotext = g_Radiotext->haveRadiotext();

	if (m_Radiotext != Radiotext)
	{
		WriteFile(RADIOTEXT, Radiotext ? "yes" : "no");
		m_Radiotext = Radiotext;
	}

	/* ----------------------------------------------------------------- */

	std::string DolbyDigital;
	if ((g_RemoteControl->current_PIDs.PIDs.selected_apid < g_RemoteControl->current_PIDs.APIDs.size()) &&
	    (g_RemoteControl->current_PIDs.APIDs[g_RemoteControl->current_PIDs.PIDs.selected_apid].is_ac3))
		DolbyDigital = "yes";
	else
		DolbyDigital = g_RemoteControl->has_ac3 ? "available" : "no";

	if (m_DolbyDigital.compare(DolbyDigital))
	{
		WriteFile(DOLBYDIGITAL, DolbyDigital);
		m_DolbyDigital = DolbyDigital;
	}

	/* ----------------------------------------------------------------- */

	CFrontend *frontend = CFEManager::getInstance()->getLiveFE();
	if (frontend)
	{
		int Tuner = frontend->getNumber() + 1;

		if (m_Tuner != Tuner)
		{
			WriteFile(TUNER, to_string(Tuner));
			m_Tuner = Tuner;
		}

		unsigned int sig = frontend->getSignalStrength() & 0xFFFF;
		int Tuner_sig = (sig & 0xFFFF) * 100 / 65535;

		if (m_Tuner_sig != Tuner_sig)
		{
			WriteFile(TUNER_SIG, to_string(Tuner_sig));
			m_Tuner_sig = Tuner_sig;
		}

		unsigned int snr = frontend->getSignalNoiseRatio() & 0xFFFF;
		int Tuner_snr = (snr & 0xFFFF) * 100 / 65535;

		if (m_Tuner_snr != Tuner_snr)
		{
			WriteFile(TUNER_SNR, to_string(Tuner_snr));
			m_Tuner_snr = Tuner_snr;
		}

		int Tuner_ber = frontend->getBitErrorRate();

		if (m_Tuner_ber != Tuner_ber)
		{
			WriteFile(TUNER_BER, to_string(Tuner_ber));
			m_Tuner_ber = Tuner_ber;
		}
	}

	/* ----------------------------------------------------------------- */

	int Volume = g_settings.current_volume;

	if (m_Volume != Volume)
	{
		WriteFile(VOLUME, to_string(Volume));
		m_Volume = Volume;
	}

	/* ----------------------------------------------------------------- */

	int ModeRec = 0;
	int ModeTshift = 0;

	int RecordMode = CRecordManager::getInstance()->GetRecordMode();
	switch (RecordMode)
	{
		case CRecordManager::RECMODE_REC_TSHIFT:
			ModeRec = 1;
			ModeTshift = 1;
			break;
		case CRecordManager::RECMODE_REC:
			ModeRec = 1;
			break;
		case CRecordManager::RECMODE_TSHIFT:
			ModeTshift = 1;
			break;
		default:
			break;
	}

	int RecordCount = CRecordManager::getInstance()->GetRecordCount();

	std::string _ModeRec = (ModeRec ? "on" : "off");
	_ModeRec += "\n" + to_string(RecordCount);

	if ((m_ModeRec != ModeRec) || (m_RecordCount != RecordCount))
	{
		WriteFile(MODE_REC, _ModeRec);
		m_ModeRec = ModeRec;
		m_RecordCount = RecordCount;
	}

	if (m_ModeTshift != ModeTshift)
	{
		WriteFile(MODE_TSHIFT, ModeTshift ? "on" : "off");
		m_ModeTshift = ModeTshift;
	}

	/* ----------------------------------------------------------------- */

	int ModeTimer = 0;

	CTimerd::TimerList timerList;
	CTimerdClient TimerdClient;

	timerList.clear();
	TimerdClient.getTimerList(timerList);

	CTimerd::TimerList::iterator timer = timerList.begin();

	for (; timer != timerList.end(); timer++)
	{
		if (timer->alarmTime > time(NULL) && (timer->eventType == CTimerd::TIMER_ZAPTO || timer->eventType == CTimerd::TIMER_RECORD))
		{
			// Nur "true", wenn irgendein timer in der zukunft liegt
			// und dieser vom typ TIMER_ZAPTO oder TIMER_RECORD ist
			ModeTimer = 1;
			break;
		}
	}

	if (m_ModeTimer != ModeTimer)
	{
		WriteFile(MODE_TIMER, ModeTimer ? "on" : "off");
		m_ModeTimer = ModeTimer;
	}

	/* ----------------------------------------------------------------- */

	int ModeEcm = 0;

	if (access("/tmp/ecm.info", F_OK) == 0)
	{
		struct stat buf;
		stat("/tmp/ecm.info", &buf);
		if (buf.st_size > 0)
			ModeEcm = 1;
	}

	if (m_ModeEcm != ModeEcm)
	{
		WriteFile(MODE_ECM, ModeEcm ? "on" : "off");
		m_ModeEcm = ModeEcm;
	}

	/* ----------------------------------------------------------------- */

	if (firstRun) //FIXME; what if module is added/removed while lcd4l is running?
	{
		for (unsigned int i = 0; i < cCA::GetInstance()->GetNumberCISlots(); i++)
			m_ModeCamPresent |= cCA::GetInstance()->ModulePresent(CA_SLOT_TYPE_CI, i);
	}

	int ModeCam = (m_ModeCamPresent && CCamManager::getInstance()->getUseCI());

	if (m_ModeCam != ModeCam)
	{
		WriteFile(MODE_CAM, ModeCam ? "on" : "off");
		m_ModeCam = ModeCam;
	}

	/* ----------------------------------------------------------------- */

	if (firstRun || newID || parseID == NeutrinoModes::mode_audio || parseID == NeutrinoModes::mode_ts)
	{
		std::string Service = "";
		int ChannelNr = 0;
		std::string Logo = LOGO_DUMMY;
		int dummy;
		int ModeLogo = 0;

		int ModeStandby	= 0;

		if (m_ModeChannel)
		{
			if (m_ModeChannel > 1)
				Service = g_RemoteControl->subChannels[g_RemoteControl->selected_subchannel].subservice_name;
			else
				Service = g_RemoteControl->getCurrentChannelName();

			g_PicViewer->GetLogoName(parseID, Service, Logo, &dummy, &dummy, CPictureViewer::LCD4LINUX, true);

			ChannelNr = CNeutrinoApp::getInstance()->channelList->getActiveChannelNumber();
		}
		else if (parseID == NeutrinoModes::mode_audio)
		{
			const CAudioMetaData meta = CAudioPlayer::getInstance()->getMetaData();
			if ((!meta.sc_station.empty()) && (CAudioPlayer::getInstance()->getState() != CBaseDec::STOP))
				Service = meta.sc_station;
			else
			{
				Service = g_Locale->getText(LOCALE_AUDIOPLAYER_NAME);

				switch (CAudioPlayer::getInstance()->getState())
				{
					case CBaseDec::REV:
						Logo = ICONSDIR "/" NEUTRINO_ICON_REW ICONSEXT;
						break;
					case CBaseDec::FF:
						Logo = ICONSDIR "/" NEUTRINO_ICON_FF ICONSEXT;
						break;
					case CBaseDec::PAUSE:
						Logo = ICONSDIR "/" NEUTRINO_ICON_PAUSE ICONSEXT;
						break;
					case CBaseDec::PLAY:
						Logo = ICONSDIR "/" NEUTRINO_ICON_PLAY ICONSEXT;
						break;
					default:
						;
				}
			}
		}
		else if (parseID == NeutrinoModes::mode_pic)
		{
			Service = g_Locale->getText(LOCALE_PICTUREVIEWER_HEAD);
		}
		else if (parseID == NeutrinoModes::mode_avinput)
		{
			//FIXME
			Logo = ICONSDIR "/" NEUTRINO_ICON_PLAY ICONSEXT;
			Service = g_Locale->getText(LOCALE_MAINMENU_AVINPUTMODE);
		}
		else if (parseID == NeutrinoModes::mode_ts)
		{
			if (ModeTshift)
				Service = g_Locale->getText(LOCALE_RECORDINGMENU_TIMESHIFT);
			else if (CMoviePlayerGui::getInstance().p_movie_info)
			{
				if (!CMoviePlayerGui::getInstance().p_movie_info->channelName.empty())
					Service = CMoviePlayerGui::getInstance().p_movie_info->channelName;
			}

			if (Service.empty())
				Service = g_Locale->getText(LOCALE_MOVIEPLAYER_HEAD);

			switch (CMoviePlayerGui::getInstance().getState())
			{
				case 6: /* rewind */
					Logo = ICONSDIR "/" NEUTRINO_ICON_REW ICONSEXT;
					break;
				case 5: /* fast forward */
					Logo = ICONSDIR "/" NEUTRINO_ICON_FF ICONSEXT;
					break;
				case 4: /* pause */
					Logo = ICONSDIR "/" NEUTRINO_ICON_PAUSE ICONSEXT;
					break;
				case 3: /* play */
					if (ModeTshift && CMoviePlayerGui::getInstance().p_movie_info) /* show channel-logo */
					{
						if (!g_PicViewer->GetLogoName(CMoviePlayerGui::getInstance().p_movie_info->channelId,
									      CMoviePlayerGui::getInstance().p_movie_info->channelName,
									      Logo, &dummy, &dummy, CPictureViewer::LCD4LINUX, true))
							Logo = ICONSDIR "/" NEUTRINO_ICON_PLAY ICONSEXT;
					}
					else /* show play-icon */
						Logo = ICONSDIR "/" NEUTRINO_ICON_PLAY ICONSEXT;
					break;
				default: /* show movieplayer-icon */
					Logo = ICONSDIR "/" NEUTRINO_ICON_MOVIEPLAYER ICONSEXT;
			}
		}
		else if (parseID == NeutrinoModes::mode_upnp)
		{
			Service = g_Locale->getText(LOCALE_UPNPBROWSER_HEAD);
		}
		else if (parseID == NeutrinoModes::mode_standby)
		{
			Service = "STANDBY";
			ModeStandby = 1;
		}

		/* --- */

		if (m_Service.compare(Service))
		{
			WriteFile(SERVICE, Service, true);
			m_Service = Service;
		}

		if (m_ChannelNr != ChannelNr)
		{
			WriteFile(CHANNELNR, to_string(ChannelNr));
			m_ChannelNr = ChannelNr;
		}

		if (m_Logo.compare(Logo))
		{
			WriteFile(LOGO, Logo);
			m_Logo = Logo;
		}

		if (Logo != LOGO_DUMMY)
			ModeLogo = 1;

		if (m_ModeLogo != ModeLogo)
		{
			WriteFile(MODE_LOGO, to_string(ModeLogo));
			m_ModeLogo = ModeLogo;
		}

		/* --- */

		std::string DisplayDriver;
		std::string DisplayRes;

		switch (g_settings.lcd4l_display_type)
		{
			case SPF800x480:
				DisplayDriver = "Samsung";
				DisplayRes = "800x480";
				break;
			case SPF800x600:
				DisplayDriver = "Samsung";
				DisplayRes = "800x600";
				break;
			case SPF1024x600:
				DisplayDriver = "Samsung";
				DisplayRes = "1024x600";
				break;
			case DPF320x240:
			default:
				DisplayDriver = "Pearl";
				DisplayRes = "320x240";
				break;
		}

		std::string DisplayType = DisplayDriver + DisplayRes;
		// Workaround for DPF
		if (g_settings.lcd4l_display_type == DPF320x240)
			DisplayType = DisplayDriver;

		std::string DisplayMode;

		if (ModeStandby)
		{
			DisplayMode = "standby";
		}
		else if ((g_settings.lcd4l_skin_radio) && (m_Mode == NeutrinoModes::mode_radio || m_Mode == NeutrinoModes::mode_webradio))
		{
			DisplayMode = "radio";
		}
		else
		{
			switch (g_settings.lcd4l_skin)
			{
				case 100:
					DisplayMode = "user";
					break;
				case 4:
					DisplayMode = "xcam";
					break;
				case 3:
					DisplayMode = "d-box2";
					break;
				case 2:
					DisplayMode = "small";
					break;
				case 1:
					DisplayMode = "large";
					break;
				case 0:
				default:
					DisplayMode = "standard";
			}
		}

		std::string Layout = DisplayType + "_" + DisplayMode;
		Layout += "\n" + DisplayDriver;
		Layout += "\n" + DisplayRes;
		Layout += "\n" + DisplayMode;

		if (m_Layout.compare(Layout))
		{
			WriteFile(LAYOUT, Layout);
			m_Layout = Layout;

			if (!firstRun)
			{
				OnBeforeRestart();
				if (exec_initscript("lcd4linux", "restart"))
					OnAfterRestart();
				else
					OnError();
			}
		}
	}

	/* ----------------------------------------------------------------- */

	std::string Event = "";
	std::string Info1 = "";
	std::string Info2 = "";
	int Progress = 0;
	char Duration[sizeof(m_Duration)] = {0};
	char Start[6] = {0};
	char End[6] = {0};

	if (m_ModeChannel)
	{
		if (CNeutrinoApp::getInstance()->getMode() == NeutrinoModes::mode_webtv || CNeutrinoApp::getInstance()->getMode() == NeutrinoModes::mode_webradio)
		{
			// FIXME: Doesn't work with timing.infobar_tv/radio=0
			if (g_InfoViewer->get_livestreamInfo1() == "RESOLUTION=1x1") // comes from best_bitrate_m3u8.lua
			{
				Event = g_InfoViewer->get_livestreamInfo2();
			}
			else
			{
				Event = g_InfoViewer->get_livestreamInfo1();
				Event += "\n" + g_InfoViewer->get_livestreamInfo2();
			}
		}

		t_channel_id channel_id = parseID & 0xFFFFFFFFFFFFULL;

		CZapitChannel *channel = CZapit::getInstance()->GetCurrentChannel();
		if (channel)
			channel_id = channel->getEpgID();

		CSectionsdClient::CurrentNextInfo CurrentNext;
		CEitManager::getInstance()->getCurrentNextServiceKey(channel_id, CurrentNext);

		if (CurrentNext.flags & CSectionsdClient::epgflags::has_current)
		{
			if (!CurrentNext.current_name.empty())
				Event = CurrentNext.current_name;

			CShortEPGData shortEpgData;
			if (CEitManager::getInstance()->getEPGidShort(CurrentNext.current_uniqueKey, &shortEpgData))
			{
				Info1 = shortEpgData.info1;
				Info2 = shortEpgData.info2;
			}

			time_t cur_duration = CurrentNext.current_zeit.dauer;
			time_t cur_start_time = CurrentNext.current_zeit.startzeit;
			if ((cur_duration > 0) && (cur_duration < 86400))
			{
				Progress = 100 * (time(NULL) - cur_start_time) / cur_duration;

				int total = cur_duration / 60;
				int done = (abs(time(NULL) - cur_start_time) + 30) / 60;
				int todo = total - done;
				if ((time(NULL) < cur_start_time) && todo >= 0)
				{
					done = 0;
					todo = cur_duration / 60;
				}
				snprintf(Duration, sizeof(Duration), "%d/%d\n%d\n%d\n%d",
					done, total,
					done,
					todo,
					total);
			}

			tm_struct = localtime(&cur_start_time);
			snprintf(Start, sizeof(Start), "%02d:%02d", tm_struct->tm_hour, tm_struct->tm_min);
		}

		if (CurrentNext.flags & CSectionsdClient::epgflags::has_next)
		{
			Event += "\n" + CurrentNext.next_name;
			time_t next_start_time = CurrentNext.next_zeit.startzeit;
			tm_struct = localtime(&next_start_time);
			snprintf(End, sizeof(End), "%02d:%02d", tm_struct->tm_hour, tm_struct->tm_min);
		}
	}
	else if (parseID == NeutrinoModes::mode_audio)
	{
		if (CAudioPlayer::getInstance()->getState() == CBaseDec::STOP)
		{
			Event = g_Locale->getText(LOCALE_AUDIOPLAYER_STOP);
			//snprintf(Duration, sizeof(Duration), "-:--");
		}
		else
		{
			const CAudioMetaData meta = CAudioPlayer::getInstance()->getMetaData();
			if (!meta.artist.empty())
				Event += meta.artist;
			if (!meta.artist.empty() && !meta.title.empty())
				Event += " - ";
			if (!meta.title.empty())
				Event += meta.title;

			if (!meta.album.empty())
				Info1 = meta.album;

			if (!meta.genre.empty())
				Info2 = meta.genre;

			time_t total = meta.total_time;
			time_t done = CAudioPlayer::getInstance()->getTimePlayed();
			time_t todo = total - done;

			if ((total > 0) && (done > 0))
			{
				Progress = 100 * done / total;

				snprintf(Duration, sizeof(Duration), "%ld:%02ld/%ld:%02ld\n%ld:%02ld\n%ld:%02ld\n%ld:%02ld",
					done / 60, done % 60, total / 60, total % 60,
					done / 60, done % 60,
					todo / 60, todo % 60,
					total / 60, total % 60);
			}
		}

		time_t sTime = time(NULL);
		tm_struct = localtime(&sTime);

		snprintf(Start, sizeof(Start), "%02d:%02d", tm_struct->tm_hour, tm_struct->tm_min);
	}
#if 0
	else if (parseID == NeutrinoModes::mode_pic)
	{
		// TODO: Event = Bildname
	}
#endif
	else if (parseID == NeutrinoModes::mode_ts)
	{
		if (CMoviePlayerGui::getInstance().p_movie_info)
		{
			if (!CMoviePlayerGui::getInstance().p_movie_info->epgTitle.empty())
				Event = CMoviePlayerGui::getInstance().p_movie_info->epgTitle;

			if (!CMoviePlayerGui::getInstance().p_movie_info->epgInfo1.empty())
				Info1 = CMoviePlayerGui::getInstance().p_movie_info->epgInfo1;

			if (!CMoviePlayerGui::getInstance().p_movie_info->epgInfo2.empty())
				Info2 = CMoviePlayerGui::getInstance().p_movie_info->epgInfo2;
		}
		else if (!CMoviePlayerGui::getInstance().GetFile().empty())
			Event = CMoviePlayerGui::getInstance().GetFile();

		if (Event.empty())
			Event = "MOVIE";

		if (!ModeTshift)
		{
			Progress = CMoviePlayerGui::getInstance().file_prozent;

			int total = CMoviePlayerGui::getInstance().GetDuration();
			int done = CMoviePlayerGui::getInstance().GetPosition();
			int todo = total - done;
			snprintf(Duration, sizeof(Duration), "%d/%d\n%d\n%d\n%d",
				done / (60 * 1000), total / (60 * 1000),
				done / (60 * 1000),
				todo / (60 * 1000),
				total / (60 * 1000));
		}

		time_t sTime = time(NULL);
		sTime -= (CMoviePlayerGui::getInstance().GetPosition() / 1000);
		tm_struct = localtime(&sTime);

		snprintf(Start, sizeof(Start), "%02d:%02d", tm_struct->tm_hour, tm_struct->tm_min);

		time_t eTime = time(NULL);
		eTime += (CMoviePlayerGui::getInstance().GetDuration() / 1000) - (CMoviePlayerGui::getInstance().GetPosition() / 1000);
		tm_struct = localtime(&eTime);

		snprintf(End, sizeof(End), "%02d:%02d", tm_struct->tm_hour, tm_struct->tm_min);
	}
#if 0
	else if (parseID == NeutrinoModes::mode_upnp)
	{
		// TODO?
	}
#endif
	/* ----------------------------------------------------------------- */

	Event += "\n"; // make sure we have at least two lines in event-file

	if (m_Event.compare(Event))
	{
		WriteFile(EVENT, Event, g_settings.lcd4l_convert);
		m_Event = Event;

		m_ParseID = 0; // reset channelid to get a possible eventlogo
	}

	if (m_Info1.compare(Info1))
	{
		WriteFile(INFO1, Info1, g_settings.lcd4l_convert);
		m_Info1 = Info1;
	}

	if (m_Info2.compare(Info2))
	{
		WriteFile(INFO2, Info2, g_settings.lcd4l_convert);
		m_Info2 = Info2;
	}

	if (m_Start.compare(Start))
	{
		WriteFile(START, (std::string)Start);
		m_Start = (std::string)Start;
	}

	if (m_End.compare(End))
	{
		WriteFile(END, (std::string)End);
		m_End = (std::string)End;
	}

	if (Progress > 100)
		Progress = 100;

	if (m_Progress != Progress)
	{
		WriteFile(PROGRESS, to_string(Progress));
		m_Progress = Progress;
	}

	if (strcmp(m_Duration, Duration))
	{
		WriteFile(DURATION, (std::string)Duration);
		strcpy(m_Duration, Duration);
	}

	if (g_settings.weather_enabled && CWeather::getInstance()->checkUpdate(firstRun))
	{
		std::string wcity = CWeather::getInstance()->getCity();
		if (m_wcity.compare(wcity))
		{
			WriteFile(WEATHER_CITY, wcity);
			m_wcity = wcity;
		}

		int forecast = CWeather::getInstance()->getForecastSize();

		std::string wtimestamp = to_string((int)CWeather::getInstance()->getCurrentTimestamp());
		for (int i = 0; i < forecast; i++) // 0 is current day
		{
			wtimestamp += "\n" + to_string(CWeather::getInstance()->getForecastWeekday(i));
		}
		if (m_wtimestamp.compare(wtimestamp))
		{
			WriteFile(WEATHER_TIMESTAMP, wtimestamp);
			m_wtimestamp = wtimestamp;
		}

		std::string wtemp = CWeather::getInstance()->getCurrentTemperature();
		for (int i = 0; i < forecast; i++) // 0 is current day
		{
			wtemp += "\n" + CWeather::getInstance()->getForecastTemperatureMin(i);
			wtemp += "|" + CWeather::getInstance()->getForecastTemperatureMax(i);
		}
		if (m_wtemp.compare(wtemp))
		{
			WriteFile(WEATHER_TEMP, wtemp);
			m_wtemp = wtemp;
		}

		std::string wwind = CWeather::getInstance()->getCurrentWindSpeed();
		wwind += "|" + CWeather::getInstance()->getCurrentWindBearing();
		wwind += "|" + CWeather::getInstance()->getCurrentWindDirection();
		for (int i = 0; i < forecast; i++) // 0 is current day
		{
			wwind += "\n" + CWeather::getInstance()->getForecastWindSpeed(i);
			wwind += "|" + CWeather::getInstance()->getForecastWindBearing(i);
			wwind += "|" + CWeather::getInstance()->getForecastWindDirection(i);
		}
		if (m_wwind.compare(wwind))
		{
			WriteFile(WEATHER_WIND, wwind);
			m_wwind = wwind;
		}

		std::string wicon = CWeather::getInstance()->getCurrentIcon();
		for (int i = 0; i < forecast; i++)
			wicon += "\n" + CWeather::getInstance()->getForecastIcon(i);
		if (m_wicon.compare(wicon))
		{
			WriteFile(WEATHER_ICON, wicon);
			m_wicon = wicon;
		}
	}
}

/* ----------------------------------------------------------------- */

bool CLCD4l::WriteFile(const char *file, std::string content, bool convert)
{
	bool ret = true;

	if (convert) // align to internal lcd4linux font
	{
		strReplace(content, "ä", "\xe4\0");
		strReplace(content, "ö", "\xf6\0");
		strReplace(content, "ü", "\xfc\0");
		strReplace(content, "Ä", "\xc4\0");
		strReplace(content, "Ö", "\xd6\0");
		strReplace(content, "Ü", "\xdc\0");
		if (g_settings.lcd4l_display_type == DPF320x240)
			strReplace(content, "ß", "\xe2\0");

		strReplace(content, "Ą", "\x41\0");
		strReplace(content, "ą", "\x61\0");
		strReplace(content, "Ć", "\x43\0");
		strReplace(content, "ć", "\x63\0");
		strReplace(content, "Ę", "\x45\0");
		strReplace(content, "ę", "\x65\0");
		strReplace(content, "Ł", "\x4c\0");
		strReplace(content, "ł", "\x6c\0");
		strReplace(content, "Ń", "\x4e\0");
		strReplace(content, "ń", "\x6e\0");
		strReplace(content, "Ó", "\x4f\0");
		strReplace(content, "ó", "\x6f\0");
		strReplace(content, "Ś", "\x53\0");
		strReplace(content, "ś", "\x73\0");
		strReplace(content, "Ź", "\x5a\0");
		strReplace(content, "ź", "\x7a\0");
		strReplace(content, "Ź", "\x5a\0");
		strReplace(content, "ż", "\x7a\0");

		strReplace(content, "é", "e");
	}

	if (FILE *f = fopen(file, "w"))
	{
		//printf("[CLCD4l] %s: %s -> %s\n", __FUNCTION__, content.c_str(), file);
		fprintf(f, "%s\n", content.c_str());
		fclose(f);
	}
	else
	{
		ret = false;
		printf("[CLCD4l] %s: %s failed!\n", __FUNCTION__, file);
	}

	return ret;
}

uint64_t CLCD4l::GetParseID()
{
	uint64_t ID = CNeutrinoApp::getInstance()->getMode();
	m_Mode = (int) ID;
	m_ModeChannel = 0;

	if (ID == NeutrinoModes::mode_tv || ID == NeutrinoModes::mode_webtv || ID == NeutrinoModes::mode_radio || ID == NeutrinoModes::mode_webradio)
	{
		if (!(g_RemoteControl->subChannels.empty()) && (g_RemoteControl->selected_subchannel > 0))
			m_ModeChannel = 2;
		else
			m_ModeChannel = 1;

		if (m_ModeChannel > 1)
			ID = g_RemoteControl->subChannels[g_RemoteControl->selected_subchannel].getChannelID();
		else
			ID = CZapit::getInstance()->GetCurrentChannelID();
	}
	//printf("[CLCD4l] %s: %llx\n", __FUNCTION__, ID);
	return ID;
}

bool CLCD4l::CompareParseID(uint64_t &i_ParseID)
{
	bool ret = false;

	i_ParseID = GetParseID();
	if (m_ParseID != i_ParseID)
	{
		//printf("[CLCD4l] %s: i_%llx <-> m_%llx\n", __FUNCTION__, i_ParseID, m_ParseID);
		ret = true;
		m_ParseID = i_ParseID;
	}
	return ret;
}

void CLCD4l::strReplace(std::string &orig, const std::string &fstr, const std::string &rstr)
{
	size_t pos = 0;
	while ((pos = orig.find(fstr, pos)) != std::string::npos)
	{
		orig.replace(pos, fstr.length(), rstr);
		pos += rstr.length();
	}
}

std::string CLCD4l::hexStr(unsigned char data)
{
	char hexstr[4];
	snprintf(hexstr, sizeof hexstr, "%02x", (int)data * 255 / 100);
	return std::string(hexstr);
}

std::string CLCD4l::hexStrA2A(unsigned char data)
{
	char hexstr[3];
	int a = 100 - data;
	int ret = a * 0xFF / 100;

	if (data == 0)
		ret = 0xFF;
	else if (data >= 100)
		ret = 0x00;

	snprintf(hexstr, sizeof hexstr, "%02x", ret);
	return std::string(hexstr);
}
