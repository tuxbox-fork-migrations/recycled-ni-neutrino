/*
	Based up Neutrino-GUI - Tuxbox-Project 
	Copyright (C) 2001 by Steffen Hehn 'McClean'

	Classes for generic GUI-related components.
	Copyright (C) 2012, 2013, Thilo Graf 'dbt'
	Copyright (C) 2012, Michael Liebmann 'micha-bbg'

	License: GPL

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Library General Public
	License as published by the Free Software Foundation; either
	version 2 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Library General Public License for more details.

	You should have received a copy of the GNU Library General Public
	License along with this library; if not, write to the
	Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
	Boston, MA  02110-1301, USA.
*/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <global.h>
#include <neutrino.h>
#include "cc.h"

using namespace std;

//abstract basic class CComponents
CComponents::CComponents()
{
	initVarBasic();
}

CComponents::~CComponents()
{
	hide();
	clearSavedScreen();
	clear();
}

void CComponents::clearSavedScreen()
{
	if (saved_screen.pixbuf)
		delete[] saved_screen.pixbuf;
	saved_screen.pixbuf = NULL;
}

void CComponents::initVarBasic()
{
	x = saved_screen.x 	= 0;
	y = saved_screen.y 	= 0;
	height 			= saved_screen.dy = CC_HEIGHT_MIN;
	width 			= saved_screen.dx = CC_WIDTH_MIN;

	col_body 		= COL_MENUCONTENT_PLUS_0;
	col_shadow 		= COL_MENUCONTENTDARK_PLUS_0;
	col_frame 		= COL_MENUCONTENT_PLUS_6;
	corner_type 		= CORNER_ALL;
	corner_rad		= 0;
	shadow			= CC_SHADOW_OFF;
	shadow_w		= SHADOW_OFFSET;
	fr_thickness		= 0;
	
	firstPaint		= true;
	is_painted		= false;
	paint_bg		= true;
	frameBuffer 		= CFrameBuffer::getInstance();
	v_fbdata.clear();
	saved_screen.pixbuf 	= NULL;
}

//paint framebuffer stuff and fill buffer
void CComponents::paintFbItems(bool do_save_bg)
{
	if (firstPaint && do_save_bg)	{
		for(size_t i=0; i<v_fbdata.size(); i++){
			if (v_fbdata[i].fbdata_type == CC_FBDATA_TYPE_BGSCREEN){
#ifdef DEBUG_CC
	printf("    [CComponents]\n    [%s - %d] firstPaint->save screen: %d, fbdata_type: %d\n", __FUNCTION__, __LINE__, firstPaint, v_fbdata[i].fbdata_type);
#endif
				saved_screen.x = v_fbdata[i].x;
				saved_screen.y = v_fbdata[i].y;
				saved_screen.dx = v_fbdata[i].dx;
				saved_screen.dy = v_fbdata[i].dy;
				clearSavedScreen();
				saved_screen.pixbuf = getScreen(saved_screen.x, saved_screen.y, saved_screen.dx, saved_screen.dy);
				firstPaint = false;
				break;
			}
		}
	}

	for(size_t i=0; i< v_fbdata.size() ;i++){
		int fbtype = v_fbdata[i].fbdata_type;
#ifdef DEBUG_CC
	printf("    [CComponents]\n    [%s - %d], fbdata_[%d] \n    x = %d\n    y = %d\n    dx = %d\n    dy = %d\n", __FUNCTION__, __LINE__, i, v_fbdata[i].x, v_fbdata[i].y, v_fbdata[i].dx, v_fbdata[i].dy);
#endif
		if (firstPaint){

			if (do_save_bg && fbtype == CC_FBDATA_TYPE_LINE)
				v_fbdata[i].pixbuf = getScreen(v_fbdata[i].x, v_fbdata[i].y, v_fbdata[i].dx, v_fbdata[i].dy);

			//ensure painting of all line fb items with saved screens
			if (fbtype == CC_FBDATA_TYPE_LINE)
				firstPaint = true;
			else
				firstPaint = false;
		}
		if (fbtype != CC_FBDATA_TYPE_BGSCREEN){
			if (fbtype == CC_FBDATA_TYPE_FRAME && v_fbdata[i].frame_thickness > 0)
				frameBuffer->paintBoxFrame(v_fbdata[i].x, v_fbdata[i].y, v_fbdata[i].dx, v_fbdata[i].dy, v_fbdata[i].frame_thickness, v_fbdata[i].color, v_fbdata[i].r);
			else if (fbtype == CC_FBDATA_TYPE_BACKGROUND)
				frameBuffer->paintBackgroundBoxRel(x, y, v_fbdata[i].dx, v_fbdata[i].dy);
			else
				frameBuffer->paintBoxRel(v_fbdata[i].x, v_fbdata[i].y, v_fbdata[i].dx, v_fbdata[i].dy, v_fbdata[i].color, v_fbdata[i].r, corner_type);
		}
	}

	is_painted = true;
}

//screen area save
inline fb_pixel_t* CComponents::getScreen(int ax, int ay, int dx, int dy)
{
	fb_pixel_t* pixbuf = new fb_pixel_t[dx * dy];
	frameBuffer->SaveScreen(ax, ay, dx, dy, pixbuf);
	return pixbuf;
}

//restore screen from buffer
inline void CComponents::hide()
{
	for(size_t i =0; i< v_fbdata.size() ;i++) {
		if (v_fbdata[i].pixbuf != NULL){
			frameBuffer->RestoreScreen(v_fbdata[i].x, v_fbdata[i].y, v_fbdata[i].dx, v_fbdata[i].dy, v_fbdata[i].pixbuf);
			delete[] v_fbdata[i].pixbuf;
			v_fbdata[i].pixbuf = NULL;
		}
	}
	v_fbdata.clear();
	is_painted = false;
}

//clean old screen buffer
inline void CComponents::clear()
{
	for(size_t i =0; i< v_fbdata.size() ;i++)
		if (v_fbdata[i].pixbuf != NULL)
			delete[] v_fbdata[i].pixbuf;
	v_fbdata.clear();
}
