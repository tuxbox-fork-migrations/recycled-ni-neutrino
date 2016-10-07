/*
	imdb

	(C) 2009-2016 NG-Team
	(C) 2016 NI-Team

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

#ifndef __imdb__
#define __imdb__

#include <unistd.h>

#include <gui/components/cc.h>

class CIMDB
{
	public:
		CIMDB();
		~CIMDB();
		static CIMDB* getInstance();

		std::string surl;
		std::string soutfile;
		std::string IMDbAPI;
		std::string posterfile;
		std::string stars_bg;
		std::string stars;

		int getIMDb(const std::string& epgTitle);
		std::string getFilename(CZapitChannel * channel, uint64_t id);
		void StringReplace(std::string &str, const std::string search, const std::string rstr);
		void cleanup();

		void getIMDbData(std::string& txt);

		bool	gotPoster() { return (access(posterfile.c_str(), F_OK) == 0); };

		virtual void getIMDbElement(std::string& element)  { element = m[element]; };

	private:
		int acc;
		std::string IMDburl;
		std::string googleIMDb(std::string searchStr);
		std::string utf82url(std::string s);
		std::string parseString(std::string search1, std::string search2, std::string str);
		std::string parseFile(std::string search1, std::string search2, const char* file, std::string firstline="", int line_offset=0);
		std::map<std::string, std::string> m;

		void	initMap(std::map<std::string, std::string>& my);
};

#endif
