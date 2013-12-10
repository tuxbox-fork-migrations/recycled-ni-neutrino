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
	General Public License for more details.

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
#include "cc_frm.h"

using namespace std;

//-------------------------------------------------------------------------------------------------------
//sub class CComponentsHeader inherit from CComponentsForm
CComponentsHeader::CComponentsHeader()
{
	//CComponentsHeader
	initVarHeader();
}

CComponentsHeader::CComponentsHeader(	const int x_pos, const int y_pos, const int w, const int h, const std::string& caption, const char* icon_name, const int buttons, bool has_shadow,
					fb_pixel_t color_frame, fb_pixel_t color_body, fb_pixel_t color_shadow)
{
	//CComponentsHeader
	initVarHeader();

	x 		= x_pos;
	y 		= y_pos;
	width 		= w;
	height 		= h > 0 ? h : height;
	shadow		= has_shadow;
	col_frame	= color_frame;
	col_body	= color_body;
	col_shadow	= color_shadow;
	
	cch_text	= caption;
	cch_icon_name	= icon_name;
	cch_buttons	= buttons;
	
	initDefaultButtons();
	initCCItems();
}

CComponentsHeader::CComponentsHeader(	const int x_pos, const int y_pos, const int w, const int h, neutrino_locale_t caption_locale, const char* icon_name, const int buttons, bool has_shadow,
					fb_pixel_t color_frame, fb_pixel_t color_body, fb_pixel_t color_shadow)
{
	//CComponentsHeader
	initVarHeader();
	
	x 		= x_pos;
	y 		= y_pos;
	width 		= w;
	height 		= h > 0 ? h : height;
	shadow		= has_shadow;
	col_frame	= color_frame;
	col_body	= color_body;
	col_shadow	= color_shadow;
	
	cch_text	= g_Locale->getText(caption_locale);
	cch_icon_name	= icon_name;
	cch_buttons	= buttons;

	initDefaultButtons();
	initCCItems();
}

void CComponentsHeader::initVarHeader()
{
	cc_item_type 		= CC_ITEMTYPE_FRM_HEADER;
	col_body 		= COL_MENUHEAD_PLUS_0;
	corner_rad		= RADIUS_LARGE,
	corner_type		= CORNER_TOP;
	
	//init header height
	cch_size_mode		= CC_HEADER_SIZE_LARGE;
	cch_font 		= g_Font[SNeutrinoSettings::FONT_TYPE_MENU_TITLE];
	height 			= cch_font->getHeight();
	
	//CComponentsHeader
	cch_icon_obj		= NULL;
	cch_text_obj		= NULL;
	cch_icon_name		= NULL;
	cch_btn_obj		= NULL;
	cch_text		= "";
	cch_col_text		= COL_MENUHEAD_TEXT;
	cch_caption_align	= CTextBox::NO_AUTO_LINEBREAK;
	cch_items_y 		= 1;
	cch_offset		= 8;
	cch_icon_x 		= cch_offset;
	cch_icon_w		= 0;
	cch_text_x		= cch_offset;
	cch_buttons		= 0;
	cch_buttons_w		= 0;
	cch_buttons_h		= 0;
	cch_buttons_space	= cch_offset;
	v_cch_btn.clear();
}

CComponentsHeader::~CComponentsHeader()
{
#ifdef DEBUG_CC
	printf("[~CComponentsHeader]   [%s - %d] delete...\n", __FUNCTION__, __LINE__);
#endif
	v_cch_btn.clear();
}

void CComponentsHeader::setCaption(const std::string& caption, const int& align_mode)
{
	cch_text		= caption;
	cch_caption_align 	= align_mode;
}

void CComponentsHeader::setCaption(neutrino_locale_t caption_locale, const int& align_mode)
{
	cch_text		= g_Locale->getText(caption_locale);
	cch_caption_align 	= align_mode;
}

void CComponentsHeader::setCaptionFont(Font* font_name)
{
	cch_font	= font_name;
	height		= std::max(height, cch_font->getHeight());
}

void CComponentsHeader::setIcon(const char* icon_name)
{
	cch_icon_name 	= icon_name;
}

void CComponentsHeader::initIcon()
{
	//init cch_icon_obj only if an icon available
	if (cch_icon_name == NULL) {
		cch_icon_w = 0;
		if (cch_icon_obj)
			delete cch_icon_obj;
		cch_icon_obj = NULL;
		return;
	}

	//create instance for cch_icon_obj
	if (cch_icon_obj == NULL){
#ifdef DEBUG_CC
	printf("    [CComponentsHeader]\n    [%s - %d] init header icon: %s\n", __FUNCTION__, __LINE__, cch_icon_name);
#endif
		cch_icon_obj = new CComponentsPicture(cch_icon_x, cch_items_y, 0, 0, cch_icon_name);
	}

	//add item only one time
	if (!cch_icon_obj->isAdded())
		addCCItem(cch_icon_obj); //icon

	//get dimensions of header icon
	int iw, ih;
	frameBuffer->getIconSize(cch_icon_name, &iw, &ih);

	//set properties for icon object
	if (cch_icon_obj){
		cch_icon_obj->setWidth(iw);
		cch_icon_obj->setHeight(ih);
		cch_icon_obj->doPaintBg(false);
		cch_icon_obj->setPictureAlign(CC_ALIGN_HOR_CENTER | CC_ALIGN_VER_CENTER);

		//set corner mode of icon item
		int cc_icon_corner_type = corner_type;
		if (corner_type == CORNER_TOP_LEFT || corner_type == CORNER_TOP)
			cc_icon_corner_type = CORNER_TOP_LEFT;
		else
			cc_icon_corner_type = CORNER_LEFT;
		cch_icon_obj->setCorner(corner_rad-fr_thickness, cc_icon_corner_type);

		//global set width of icon object
		cch_icon_w = cch_icon_obj->getWidth();

		//global adapt height
		height = max(height, cch_icon_obj->getHeight());

		//re-align height of icon object
		cch_icon_obj->setHeight(height);
	}
}

void CComponentsHeader::addButtonIcon(const std::string& button_name)
{
	v_cch_btn.push_back(button_name);
	initButtons();
}

void CComponentsHeader::removeButtonIcons()
{
	v_cch_btn.clear();
	cch_btn_obj->removeAllIcons();
	initButtons();
}

void CComponentsHeader::initDefaultButtons()
{
	if (cch_buttons & CC_BTN_EXIT)
		v_cch_btn.push_back(NEUTRINO_ICON_BUTTON_HOME);
	if (cch_buttons & CC_BTN_HELP)
		v_cch_btn.push_back(NEUTRINO_ICON_BUTTON_HELP);
	if (cch_buttons & CC_BTN_INFO)
		v_cch_btn.push_back(NEUTRINO_ICON_BUTTON_INFO);
	if (cch_buttons & CC_BTN_MENU)
		v_cch_btn.push_back(NEUTRINO_ICON_BUTTON_MENU);
#ifdef DEBUG_CC
	printf("[CComponentsHeader]  %s added %d default buttons...\n", __FUNCTION__, v_cch_btn.size());
#endif
}

void CComponentsHeader::setDefaultButtons(const int buttons)
{
	cch_buttons = buttons;
	v_cch_btn.clear();
	initDefaultButtons();
}

// calculate minimal width of icon form
void CComponentsHeader::initButtonFormSize()
{
	cch_buttons_w = 0;
	cch_buttons_h = 0;

	if (cch_btn_obj == NULL)
		return;
	
	for(size_t i=0; i<v_cch_btn.size(); i++){
		int bw, bh;
		frameBuffer->getIconSize(v_cch_btn[i].c_str(), &bw, &bh);
		cch_buttons_w += (bw + cch_buttons_space);
		cch_buttons_h = std::max(cch_buttons_h, bh);
	}
	cch_buttons_w -= cch_buttons_space;
}

void CComponentsHeader::initButtons()
{
	//exit if no button defined
	if (v_cch_btn.empty()){
		if (cch_btn_obj)
			delete cch_btn_obj;
		cch_btn_obj = NULL;
		return;
	}
	
	initButtonFormSize();

	if (cch_btn_obj == NULL){
		cch_btn_obj = new CComponentsIconForm();
#ifdef DEBUG_CC
	printf("    [CComponentsHeader]\n    [%s - %d] init header buttons...\n", __FUNCTION__, __LINE__);
#endif
	}

	//add button form only one time
	if (!cch_btn_obj->isAdded())
		addCCItem(cch_btn_obj); //buttons

	//set button form properties
	if (cch_btn_obj){
		cch_btn_obj->setDimensionsAll(width-cch_offset-cch_buttons_w, cch_items_y, cch_buttons_w, cch_buttons_h);
		cch_btn_obj->doPaintBg(false);
		cch_btn_obj->setIconOffset(cch_buttons_space);
		cch_btn_obj->setIconAlign(CComponentsIconForm::CC_ICONS_FRM_ALIGN_RIGHT);
		cch_btn_obj->removeAllIcons();
		cch_btn_obj->addIcon(v_cch_btn);

		//set corner mode of button item
		int cc_btn_corner_type = corner_type;
		if (corner_type == CORNER_TOP_RIGHT || corner_type == CORNER_TOP)
			cc_btn_corner_type = CORNER_TOP_RIGHT;
		else
			cc_btn_corner_type = CORNER_RIGHT;
		cch_btn_obj->setCorner(corner_rad-fr_thickness, cc_btn_corner_type);

		//global adapt height
		height = max(height, cch_btn_obj->getHeight());

		//re-align height of button object
		cch_btn_obj->setHeight(height);

		//re-align height of icon object
		if (cch_icon_obj)
			cch_icon_obj->setHeight(height);
	}
}

void CComponentsHeader::initCaption()
{
	//recalc header text position if header icon is defined
	int cc_text_w = 0;
	if (cch_icon_name != NULL){
		cch_text_x = cch_icon_x+cch_icon_w+cch_offset;
	}

	//calc width of text object in header
	cc_text_w = width-cch_text_x-cch_offset;
	if (cch_buttons_w)
		cc_text_w -= cch_buttons_w-cch_offset;

	//create cch_text_obj and add to collection
	if (cch_text_obj == NULL){
#ifdef DEBUG_CC
	printf("    [CComponentsHeader]\n    [%s - %d] init header text: %s [ x %d w %d ]\n", __FUNCTION__, __LINE__, cch_text.c_str(), cch_text_x, cc_text_w);
#endif
		cch_text_obj = new CComponentsText();
	}

	//add text item
	if (!cch_text_obj->isAdded())
		addCCItem(cch_text_obj); //text

	//set header text properties
	if (cch_text_obj){
			//set alignment of text item in dependency from text alignment
		if (cch_caption_align == CTextBox::CENTER)
			cch_text_x = CC_CENTERED;
		cch_text_obj->setDimensionsAll(cch_text_x, cch_items_y, cc_text_w, height);
		cch_text_obj->doPaintBg(false);
		cch_text_obj->setText(cch_text, cch_caption_align, cch_font);
		cch_text_obj->forceTextPaint(); //here required
		cch_text_obj->setTextColor(cch_col_text);
		cch_text_obj->setColorBody(col_body);

		//corner of text item
		cch_text_obj->setCorner(corner_rad-fr_thickness, corner_type);

		/*
		   global adapt height not needed here again
		   because this object is initialized at last
		*/
		//height = max(height, cch_text_obj->getHeight());
	}
}

void CComponentsHeader::initCCItems()
{
	//set size
	cch_font = (cch_size_mode == CC_HEADER_SIZE_LARGE? g_Font[SNeutrinoSettings::FONT_TYPE_MENU_TITLE] : g_Font[SNeutrinoSettings::FONT_TYPE_MENU]);
	height = cch_font->getHeight();
	
	//init icon
	initIcon();

	//init buttons
	initButtons();

	//init text
	initCaption();
}
	
void CComponentsHeader::paint(bool do_save_bg)
{
	//prepare items
	initCCItems();
	
	//paint form contents
	paintForm(do_save_bg);
}
