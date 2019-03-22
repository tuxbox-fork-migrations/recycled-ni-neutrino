/*
	Copyright (C) 2017,2018,2019 TangoCash

	“Powered by Dark Sky” https://darksky.net/poweredby/

	License: GPLv2

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation;

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifndef __WEATHER__
#define __WEATHER__

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <string>
#include <time.h>
#include <vector>

#include "system/settings.h"
#include "system/helpers.h"

#include <gui/components/cc.h>

struct current_data
{
	std::string icon;
	float temperature;

	current_data():
		icon(""),
		temperature(0)
	{}
};

typedef struct
{
	time_t timestamp;
	std::string icon;
	float temperatureMin;
	float temperatureMax;
} forecast_data;

class CWeather
{
	private:
		std::string coords;
		std::string city;
		std::string timezone;
		current_data current;
		std::vector<forecast_data> v_forecast;
		CComponentsForm *form;
		std::string key;
		bool GetWeatherDetails();
		time_t last_time;

	public:
		static CWeather *getInstance();
		CWeather();
		~CWeather();
		bool checkUpdate(bool forceUpdate = false);
		void setCoords(std::string new_coords, std::string new_city = "Unknown");

		std::string getCity()
		{
			return city;
		};
		std::string getCurrentTemperature()
		{
			return to_string((int)(current.temperature + 0.5));
		};
		std::string getCurrentIcon()
		{
			return ICONSDIR"/weather/" + current.icon;
		};
		std::string getForecastTemperatureMax(int i = 0)
		{
			return to_string((int)(v_forecast[i].temperatureMax + 0.5));
		};
		std::string getForecastIcon(int i = 0)
		{
			return ICONSDIR"/weather/" + v_forecast[i].icon;
		};

		void show(int x = 50, int y = 50);
		void hide();
};

#endif
