/*
	Based up Neutrino-GUI - Tuxbox-Project
	Copyright (C) 2001 by Steffen Hehn 'McClean'

	Class for signalbar based up CComponent classes.
	Copyright (C) 2013, Thilo Graf 'dbt'

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

#ifndef __SIGNALBARS_H__
#define __SIGNALBARS_H__


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <global.h>
#include <neutrino.h>
#include <gui/components/cc_frm.h>
#include <gui/components/cc_item_progressbar.h>
#include <gui/components/cc_item_text.h>
#include <zapit/include/zapit/frontend_c.h>


/// Basic class for signalbars
/*!
Basic attributes and member functions for items.
These class provides basic attributes and members to show frontend values in signalbars.
CSignalBar() and their sub classes based up CComponentsForm() and are usable like other CComponentsItems()

CSignalBar() is intended to show signal rate.
*/

class CSignalBar : public CComponentsForm
{
	protected:
		///object: current frontend
		CFrontend	 	*sb_frontend;
		///object: scale bar
		CProgressBar 		*sb_scale;
		///object: value caption
		CComponentsLabel 	*sb_vlbl;
		///object: caption for signal name
		CComponentsLabel 	*sb_lbl;
		///object: current font
		Font			*sb_font;
		///property: text color, see also setTextColor()
		fb_pixel_t 		sb_caption_color;

		///property: height of items
		int sb_item_height;
		///property: width of progressbar
		int sb_scale_width;
		///property: height of progressbar
		int sb_scale_height;
		///property: width of value caption
		int sb_vlbl_width;
		///property: width of caption
		int sb_lbl_width;

		///cache last assingned signal value
		int sb_lastsig;
		///current signal value
		uint16_t sb_signal;

		///initialize all needed basich attributes and objects
		void initVarSigBar();
		///initianlize position and dimensions of signalbar container
		void initDimensions();
		///initialize scale object
		void initSBarScale();
		///initialize value caption object, this contains the value of current frontend data, signal or noise rate
		void initSBarValue();
		///initialize caption object, this contains the unit (e.g %) or name of value (e.g. SIG)
		void initSBarName();

		///initialize all required objects at once, see also Refresh()
		void initSBItems();

		///property: contains the name of signal type in the caption object, see also setName()
		std::string sb_name;

	public:
		CSignalBar();
		///basic component class constructor for signal.
		CSignalBar(const int& xpos, const int& ypos, const int& w, const int& h, CFrontend *frontend_ref);

		///assigns the current used frontend, simplified a tuner object, see frontend_c.h
		virtual void setFrontEnd(CFrontend *frontend_ref){sb_frontend = frontend_ref;};
		///assigns font for caption
		virtual void setTextFont(Font* font_text){sb_font = font_text;};
		///sets the caption color, see also property 'sb_caption_color'
		virtual void setTextColor(const fb_pixel_t& caption_color){ sb_caption_color = caption_color;};
		///assigns the height of scale
		virtual void setScaleHeight(const int& scale_height){sb_scale_height = scale_height;};
		///assigns the name of signal value in the caption object, see also sb_name
		virtual void setName(const std::string& name){sb_name = name;};

		///returns the scale object, type = CProgressBar*
		virtual CProgressBar* getScaleObject(){return sb_scale;};
		///returns the caption object, type = CComponentsLabel*
		virtual CComponentsLabel* getLabelObject(){return sb_lbl;};

		///refresh current item properties, use this before Repaint().
		void Refresh();
		///reinitialize current signal values and paint new values, required after Refresh()
		virtual void Repaint();

};

/// Sub class of CSignalBar()
/*!
This class use basic attributes and members from CSignalBar() to show frontend values.

CSignalNoiseRatioBar() is intended to show signal noise ratio value.
*/

class CSignalNoiseRatioBar : public CSignalBar
{
	protected:
		///initialize all needed basic attributes and objects
		void initVarSnrBar();

	public:
		CSignalNoiseRatioBar();
		///basic component class constructor for signal noise ratio.
		CSignalNoiseRatioBar(const int& xpos, const int& ypos, const int& w, const int& h, CFrontend *frontend_ref);

		///refresh current item properties, use this before repaintSignalBar().
		void Refresh();
};

/// Class CSignalBox() provides CSignalBar(), CSignalNoiseRatioBar() scales at once.
/*!
Provides basic attributes and member functions for CComponentItems in
additional of CSignalBar()- and CSignalNoiseRatioBar()-objects.


To add a signalbox object to your code add this to a header file:
#include <gui/widget/signalbars.h>

class CSampleClass
{
	private:
		//other stuff;
		CSignalBox * signalbox;

	public:
		CSampleClass();
		~CSampleClass();
		void showSNR();

		//other stuff;

};


//add this to your costructor into the code file:
CSampleClass::CSampleClass()
{
	//other stuff;
	signalbox = NULL;
}

CStreamInfo2::~CStreamInfo2 ()
{
	//other stuff to clean;
	delete signalbox;
	//other stuff to clean;
}

void CSampleClass::showSNR()
{
	if (signalbox == NULL){
		signalbox = new CSignalBox(10, 100, 500, 38, frontend);
		signalbox->setCornerRadius(0);
// 		signalbox->setColorBody(COL_BLACK);
		signalbox->setColorBody(COL_MENUHEAD_PLUS_0);
		signalbox->doPaintBg(false);
//if you want to add the object to a CC-Container (e.g. CComponentsWindow()), remove this line:
		signalbox->paint(false);
//and add this lines:
//		if (!isAdded(signalbox))
//			addCCItem(signalbox);
//Note: signal box object deallocate together with the CC-Container!
 	}
	else{
		signalbox->Refresh();
		signalbox->Repaint();
 	}
 }

void CSampleClass::hide ()
{
	//other code;

//Note: not required if signalbox is added to a CC-Container!
	signalbox->hide(true);
	delete signalbox;
	signalbox = NULL;

	//other code;
}

*/

class CSignalBox : public CComponentsForm
{
	protected:
		///object: current frontend
		CFrontend	 	*sbx_frontend;
		///object: current signalbar
		CSignalBar		*sbar;
		///object: current signal noise ratio bar
		CSignalNoiseRatioBar	*snrbar;
		///property: height of signalbars
		int sbx_bar_height;
		///property: width of signalbars
		int sbx_bar_width;
		///property: x position of signalbars
		int sbx_bar_x;
		///property: text color, see also setTextColor()
		fb_pixel_t 		sbx_caption_color;
		///initialize all needed basic attributes and objects
		void initVarSigBox();

		void initSignalItems();

	public:
		///class constructor for signal noise ratio.
		CSignalBox(const int& xpos, const int& ypos, const int& w, const int& h, CFrontend *frontend_ref);

		///returns the signal object, type = CSignalBar*
		virtual CSignalBar* getScaleObject(){return sbar;};
		///returns the signal noise ratio object, type = CSignalNoiseRatioBar*
		virtual CSignalNoiseRatioBar* getLabelObject(){return snrbar;};

		///sets the caption color of signalbars, see also property 'sbx_caption_color'
		void setTextColor(const fb_pixel_t& caption_color){ sbx_caption_color = caption_color;};
		///sets the caption color of signalbars, see also property 'sbx_caption_color'
		fb_pixel_t getTextColor(){return sbx_caption_color;};

		///refresh all current snr value, use this before paint().
		void Refresh();

		///paint items with new values, required after Refresh()
		void Repaint();
};

#endif
