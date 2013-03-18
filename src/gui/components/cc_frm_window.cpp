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
#include <driver/screen_max.h>

using namespace std;

//-------------------------------------------------------------------------------------------------------
//sub class CComponentsWindow inherit from CComponentsForm
CComponentsWindow::CComponentsWindow()
{
	initVarWindow();
}

CComponentsWindow::~CComponentsWindow()
{
#ifdef DEBUG_CC
	printf("[~CComponentsWindow]   [%s - %d] delete...\n", __FUNCTION__, __LINE__);
#endif
	cleanCCForm();
}

void CComponentsWindow::initVarWindow()
{
	//CComponentsForm
	initVarForm();
	cc_item_type 	= CC_ITEMTYPE_FRM_WINDOW;
	
	ccw_head 	= NULL;
	ccw_caption 	= "";
	ccw_icon_name	= NULL;

	//using current screen settings for default dimensions
	width = frameBuffer->getScreenWidth();
	height = frameBuffer->getScreenHeight();
	x=getScreenStartX(width);
	y=getScreenStartY(height);
	
	setShadowOnOff(true);
}

void CComponentsWindow::setWindowCaption(neutrino_locale_t locale_text)
{
	ccw_caption = g_Locale->getText(locale_text);
}

void CComponentsWindow::initHeader()
{
	if (ccw_head == NULL){
		ccw_head = new CComponentsHeader();
		initHeader();
		//add header item only one time
		addCCItem(ccw_head);
	}

	//set header properties
	if (ccw_head){
		ccw_head->setXPos(0);
		ccw_head->setYPos(0);
		ccw_head->setWidth(width);
		ccw_head->setHeaderIcon(ccw_icon_name);
		ccw_head->setHeaderText(ccw_caption);
	}
}

void CComponentsWindow::initCCWItems()
{
#ifdef DEBUG_CC
	printf("[CComponentsWindow]   [%s - %d] init items...\n", __FUNCTION__, __LINE__);
#endif
	initHeader();
}

void CComponentsWindow::paint(bool do_save_bg)
{
	//prepare items before paint
	initCCWItems();
	
	//paint form contents
	paintForm(do_save_bg);
}
