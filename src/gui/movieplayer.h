/*
  Neutrino-GUI  -   DBoxII-Project

  Copyright (C) 2003,2004 gagga
  Homepage: http://www.giggo.de/dbox

  Kommentar:

  Diese GUI wurde von Grund auf neu programmiert und sollte nun vom
  Aufbau und auch den Ausbaumoeglichkeiten gut aussehen. Neutrino basiert
  auf der Client-Server Idee, diese GUI ist also von der direkten DBox-
  Steuerung getrennt. Diese wird dann von Daemons uebernommen.


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

#ifndef __movieplayergui__
#define __movieplayergui__

#include <config.h>
#include <configfile.h>
#include <driver/framebuffer.h>
#include <gui/filebrowser.h>
#include <gui/bookmarkmanager.h>
#include <gui/widget/menue.h>
#include <gui/moviebrowser.h>
#include <gui/movieinfo.h>
#include <gui/widget/hintbox.h>
#include <gui/timeosd.h>
#include <driver/record.h>
#include <playback.h>

#include <stdio.h>

#include <string>
#include <vector>

class CMoviePlayerGui : public CMenuTarget
{
 public:
	enum state
		{
		    STOPPED     =  0,
		    PREPARING   =  1,
		    STREAMERROR =  2,
		    PLAY        =  3,
		    PAUSE       =  4,
		    FF          =  5,
		    REW         =  6
		};

 private:
	CFrameBuffer * frameBuffer;
	int            m_LastMode;	

	std::string	full_name;
	std::string	file_name;
	std::string    	currentaudioname;
	bool		playing;
	CMoviePlayerGui::state playstate;
	int speed;
	int startposition;
	int position;
	int duration;
	CTimeOSD FileTime;

	unsigned short numpida;
	unsigned short vpid;
	unsigned short vtype;
	std::string    language[MAX_PLAYBACK_PIDS];
	unsigned short apids[MAX_PLAYBACK_PIDS];
	unsigned short ac3flags[MAX_PLAYBACK_PIDS];
	unsigned short currentapid, currentac3;

	/* subtitles vars */
	unsigned short numsubs;
	std::string    slanguage[MAX_PLAYBACK_PIDS];
	unsigned short spids[MAX_PLAYBACK_PIDS];
	unsigned short sub_supported[MAX_PLAYBACK_PIDS];
	int currentspid;
	int min_x, min_y, max_x, max_y;
	time_t end_time;
	bool ext_subs;

	/* playback from MB */
	bool isMovieBrowser;
	bool isHTTP;
	bool isUPNP;
	bool showStartingHint;
	CMovieBrowser* moviebrowser;
	MI_MOVIE_INFO * p_movie_info;
	const static short MOVIE_HINT_BOX_TIMER = 5;	// time to show bookmark hints in seconds

	/* playback from file */
	bool is_file_player;
	bool iso_file;
	CFileBrowser * filebrowser;
	CFileFilter tsfilefilter;
	std::string Path_local;
	int menu_ret;
	bool autoshot_done;

	/* playback from bookmark */
	CBookmarkManager * bookmarkmanager;
	bool isBookmark;

	cPlayback *playback;
	static CMoviePlayerGui* instance_mp;

	void Init(void);
	void PlayFile();
	void cutNeutrino();
	void restoreNeutrino();

	void showHelpTS(void);
	void callInfoViewer(/*const int duration, const int pos*/);
	void fillPids();
	bool getAudioName(int pid, std::string &apidtitle);
	void selectAudioPid(bool file_player);
	void getCurrentAudioName( bool file_player, std::string &audioname);
	void addAudioFormat(int count, std::string &apidtitle, bool& enabled );

	void handleMovieBrowser(neutrino_msg_t msg, int position = 0);
	bool SelectFile();
	void updateLcd();

	void selectSubtitle();
	bool convertSubtitle(std::string &text);
	void showSubtitle(neutrino_msg_data_t data);
	void clearSubtitle();
	void selectChapter();
	void selectAutoLang();
	void parsePlaylist(CFile *file);
	bool mountIso(CFile *file);
	void makeFilename();
	void makeScreenShot(bool autoshot = false, bool forcover = false);

	void Cleanup();
	static void *ShowStartHint(void *arg);

	CMoviePlayerGui(const CMoviePlayerGui&) {};
	CMoviePlayerGui();

 public:
	~CMoviePlayerGui();

	static CMoviePlayerGui& getInstance();

	int exec(CMenuTarget* parent, const std::string & actionKey);
	bool Playing() { return playing; };
	bool osdTimeVisible() { return FileTime.IsVisible(); };
	std::string CurrentAudioName() { return currentaudioname; };
	int GetSpeed() { return speed; }
	int GetPosition() { return position; }
	int GetDuration() { return duration; }
	void UpdatePosition();
	int timeshift;
	int file_prozent;
	void SetFile(std::string &name, std::string &file) { file_name = name; full_name = file; }
};

#endif
