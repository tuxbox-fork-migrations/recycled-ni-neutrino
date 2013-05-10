/*
	Based up Neutrino-GUI - Tuxbox-Project 
	Copyright (C) 2001 by Steffen Hehn 'McClean'

	Classes for generic GUI-related components.
	Copyright (C) 2012, 2013, Thilo Graf 'dbt'
	Copyright (C) 2012, Michael Liebmann 'micha-bbg'

	License: GPL

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public
	License as published by the Free Software Foundation; either
	version 2 of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Library General Public License for more details.

	You should have received a copy of the GNU General Public
	License along with this program; if not, write to the
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

//sub class CComponentsInfoBox from CComponentsItem
CComponentsInfoBox::CComponentsInfoBox()
{
	//CComponentsInfoBox
	initVarInfobox();
}

CComponentsInfoBox::CComponentsInfoBox(const int x_pos, const int y_pos, const int w, const int h,
				       std::string info_text, const int mode, Font* font_text,
				       bool has_shadow,
				       fb_pixel_t color_text, fb_pixel_t color_frame, fb_pixel_t color_body, fb_pixel_t color_shadow)
{
	//CComponentsInfoBox
	initVarInfobox();
	
	x 		= x_pos;
	y 		= y_pos;
	width 		= w;
	height	 	= h;
	shadow		= has_shadow;
	col_frame 	= color_frame;
	col_body	= color_body;
	col_shadow	= color_shadow;

	ct_text 	= info_text;
	ct_text_mode	= mode;
	ct_font		= font_text;
	ct_col_text	= color_text;
}

CComponentsInfoBox::~CComponentsInfoBox()
{
	hide();
	clearSavedScreen();
	clearCCText();
	delete pic;
	delete cctext;
	clear();
}

void CComponentsInfoBox::initVarInfobox()
{
	//CComponents, CComponentsItem,  CComponentsText
	initVarText();
	cc_item_type = CC_ITEMTYPE_TEXT_INFOBOX;

	//CComponentsInfoBox
	pic 		= NULL;
	cctext		= NULL;
	pic_name	= "";
	x_offset	= 10;
	x_text		= x+fr_thickness+x_offset;;

}

void CComponentsInfoBox::paintPicture()
{
	//ensure empty pic object
	if (pic)
		delete pic;
	pic = NULL;

	//exit if no image definied
	if (pic_name == "")
		return;
	
	//init pic object and set icon paint position
	pic = new CComponentsPicture(x+fr_thickness+x_offset, y+fr_thickness/*+y_offset*/, 0, 0, "");
	
	//define icon
	pic->setPicture(pic_name);
	
	//fit icon into infobox
	pic->setHeight(height-2*fr_thickness);
	pic->setColorBody(col_body);
	
	pic->paint(CC_SAVE_SCREEN_NO);	
}

void CComponentsInfoBox::paint(bool do_save_bg)
{
	paintInit(do_save_bg);
	paintPicture();

	//define text x position
	x_text = x+fr_thickness+x_offset;
	
	//set text to the left border if picture is not painted
	if ((pic) && (pic->isPicPainted())){
		int pic_w = pic->getWidth();
		x_text += pic_w+x_offset;
	}

	//set text and paint text lines
 	if (!ct_text.empty()){
 		if (cctext)
			delete cctext;
		
		cctext = new CComponentsText();
		cctext->setText(ct_text, ct_text_mode, ct_font);
		cctext->doPaintTextBoxBg(ct_paint_textbg);
		cctext->doPaintBg(false);
		cctext->setTextColor(ct_col_text);
		cctext->setDimensionsAll(x_text, y+fr_thickness, width-(x_text-x+x_offset+fr_thickness), height-2*fr_thickness);
 		cctext->paint(CC_SAVE_SCREEN_NO);
	}
}
