/*
	Based up Neutrino-GUI - Tuxbox-Project
	Copyright (C) 2001 by Steffen Hehn 'McClean

	Hintbox based up initial code by
	Copyright (C) 2003 Ralf Gandy 'thegoodguy'
	Copyright (C) 2004 Sven Traenkle 'zwen'
	Copyright (C) 2008-2009, 2011, 2013 Stefan Seyfried

	Implementation of CComponent Window class.
	Copyright (C) 2014-2016 Thilo Graf 'dbt'

	License: GPL

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public
	License as published by the Free Software Foundation; either
	version 2 of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef __C_HINTBOX__
#define __C_HINTBOX__

#include <gui/components/cc.h>

#define HINTBOX_MIN_WIDTH	320 // scaled in init
#define HINTBOX_MIN_HEIGHT	CFrameBuffer::getInstance()->scale2Res(125)
#define HINTBOX_MAX_HEIGHT	CFrameBuffer::getInstance()->scale2Res(520)

#define HINTBOX_DEFAULT_TIMEOUT g_settings.timing[SNeutrinoSettings::TIMING_POPUP_MESSAGES]
#define NO_TIMEOUT 0
#define DEFAULT_TIMEOUT -1
//frame around hint container as indent
#define W_FRAME OFFSET_INNER_MID
//frame color around hint/message box
#define HINTBOX_DEFAULT_FRAME_COLOR COL_FRAME
#define HINTBOX_DEFAULT_FRAME_WIDTH OFFSET_INNER_SMALL
#define TIMEOUT_BAR_HEIGHT  OFFSET_SHADOW/2

#define DEFAULT_HINTBOX_TEXT_MODE (CTextBox::CENTER)
#define DEFAULT_HEADER_ICON NEUTRINO_ICON_INFO

//! Sub class of CComponentsWindow. Shows a window as a hintbox with text and optional icon beside of text.
/*!
CHintBox provides a small window with header and a text item,
optional you can add an icon in the header and/or beside left of
text and context buttons on the right site of header.
*/

class CHintBox : public CComponentsWindow
{
	protected:
		int y_hint_obj;
		int h_hint_obj;
		int w_indentation;
		bool enable_txt_scroll;
		sigc::slot0<void> sl_tbar_on_timer;
		bool enable_timeout_bar;

		Font* hb_font;

		///timeout value, see also setTimeOut()
		int timeout;

		///timeout bar
		CProgressBar *timeout_pb;
		CComponentsTimer *timeout_pb_timer;

		///scroll handler, default down and for the 1st hint item (=0), NOTE: exec() must be called! see also scroll_down()/scroll_up()
		void Scroll(bool down, const uint& hint_id = 0);

		///main init handler
		void init(	const std::string& Text,
				const int& Width,
				const std::string& Picon,
				const int& header_buttons,
				const int& text_mode,
				const int& indent,
				const fb_pixel_t& color_frame,
				const fb_pixel_t& color_body,
				const fb_pixel_t& color_shadow,
				const int& frame_width);

		virtual void ReSize();
		void showTimeOutBar(){initTimeOutBar();}
		int getMaxWidth(const std::string& Text, const std::string& Title, Font *font, const int& minWidth);

	public:
		/**CHintBox Constructor
		* @param[in]	Caption
		* 	@li 	expects type neutrino_locale_t with locale entry from /system/locals.h
		* @param[in]	Text
		* 	@li 	expects type const char*, this is the message text inside the window, text is UTF-8 encoded
		* @param[in]	Width
		* 	@li 	optional: expects type int, defines box width, default value = HINTBOX_MIN_WIDTH
		* @param[in]	Icon
		* 	@li 	optional: expects type const char*, defines the icon name on the left side of titlebar, default = DEFAULT_HEADER_ICON
		* @param[in]	Picon
		* 	@li 	optional: expects type const char*, defines the picon name on the left side of message text, default = NULL (non Icon)
		* 		special case: If picon == NEUTRINO_ICON_LOADER, then the animated loader icon known from CHourGlass object will be painted.
		* 	@see	CHourGlass()
		* @param[in]	header_buttons
		* 	@li 	optional: expects type int, defines the icon name on the right side of titlebar, default = 0 (non Icon)
		* 	@see	class CComponentsWindow()
		* @param[in]	text_mode
		* 	@li 	optional: expects type int, defines the text modes for embedded text lines
		* 		Possible Modes defined in /gui/widget/textbox.h
		* 		AUTO_WIDTH
		* 		AUTO_HIGH
		* 		SCROLL
		* 		CENTER
		* 		RIGHT
		* 		TOP
		* 		BOTTOM
		* 		NO_AUTO_LINEBREAK
		* 		AUTO_LINEBREAK_NO_BREAKCHARS
		* @param[in]	indent
		* 	@li	optional: expects type int, defines indent of text
		*
		* 	@see	classes CComponentsText(), CTextBox()
		*/
		CHintBox(	const neutrino_locale_t Caption,
				const char * const Text,
				const int Width = HINTBOX_MIN_WIDTH,
				const char * const Icon = DEFAULT_HEADER_ICON,
				const char * const Picon = NULL,
				const int& header_buttons = 0,
				const int& text_mode = DEFAULT_HINTBOX_TEXT_MODE,
				const int& indent = W_FRAME,
				const fb_pixel_t& color_frame = HINTBOX_DEFAULT_FRAME_COLOR,
				const fb_pixel_t& color_body = COL_MENUCONTENT_PLUS_0,
				const fb_pixel_t& color_shadow = COL_SHADOW_PLUS_0,
				const int& frame_width = HINTBOX_DEFAULT_FRAME_WIDTH);

		/**CHintBox Constructor
		* @param[in]	Caption
		* 	@li 	expects type const char*
		* 	@see	for other parameters take a look to basic class CHintBox()
		*/
		CHintBox(	const char * const Caption,
				const char * const Text,
				const int Width = HINTBOX_MIN_WIDTH,
				const char * const Icon = DEFAULT_HEADER_ICON,
				const char * const Picon = NULL,
				const int& header_buttons = 0,
				const int& text_mode = DEFAULT_HINTBOX_TEXT_MODE,
				const int& indent = W_FRAME,
				const fb_pixel_t& color_frame = HINTBOX_DEFAULT_FRAME_COLOR,
				const fb_pixel_t& color_body = COL_MENUCONTENT_PLUS_0,
				const fb_pixel_t& color_shadow = COL_SHADOW_PLUS_0,
				const int& frame_width = HINTBOX_DEFAULT_FRAME_WIDTH);

		/**CHintBox Constructor
		* @param[in]	Caption
		* 	@li 	expects type neutrino_locale_t with locale entry from /system/locals.h
		* @param[in]	Text
		* 	@li 	expects type neutrino_locale_t with locale entry from /system/locals.h
		* 	@see	for other parameters take a look to basic class CHintBox()
		*/
		CHintBox(	const neutrino_locale_t Caption,
				const neutrino_locale_t Text,
				const int Width = HINTBOX_MIN_WIDTH,
				const char * const Icon = DEFAULT_HEADER_ICON,
				const char * const Picon = NULL,
				const int& header_buttons = 0,
				const int& text_mode = DEFAULT_HINTBOX_TEXT_MODE,
				const int& indent = W_FRAME,
				const fb_pixel_t& color_frame = HINTBOX_DEFAULT_FRAME_COLOR,
				const fb_pixel_t& color_body = COL_MENUCONTENT_PLUS_0,
				const fb_pixel_t& color_shadow = COL_SHADOW_PLUS_0,
				const int& frame_width = HINTBOX_DEFAULT_FRAME_WIDTH);

		/**CHintBox Constructor
		* @param[in]	Caption
		* 	@li 	expects type const char*
		* @param[in]	Text
		* 	@li 	expects type neutrino_locale_t with locale entry from /system/locals.h
		* 	@see	for other parameters take a look to basic class CHintBox()
		*/
		CHintBox(	const char * const Caption,
				const neutrino_locale_t Text,
				const int Width = HINTBOX_MIN_WIDTH,
				const char * const Icon = DEFAULT_HEADER_ICON,
				const char * const Picon = NULL,
				const int& header_buttons = 0,
				const int& text_mode = DEFAULT_HINTBOX_TEXT_MODE,
				const int& indent = W_FRAME,
				const fb_pixel_t& color_frame = HINTBOX_DEFAULT_FRAME_COLOR,
				const fb_pixel_t& color_body = COL_MENUCONTENT_PLUS_0,
				const fb_pixel_t& color_shadow = COL_SHADOW_PLUS_0,
				const int& frame_width = HINTBOX_DEFAULT_FRAME_WIDTH);

		virtual ~CHintBox();
		/**
		* exec caller
		* @return	int
		*/
		int exec();

		/**
		* Defines timeout for message window.
		* Timeout is enabled with parameter1 = DEFAULT_TIMEOUT (-1) or any other value > 0
		* To disable timeout use NO_TIMEOUT (0)
		* @param[in]	Timeout as int as seconds
		* @param[in]	enable_Timeout_Bar as bool
		*/
		void setTimeOut(const int& Timeout, const bool& enable_Timeout_Bar){timeout = Timeout; enable_timeout_bar = enable_Timeout_Bar;}

		/**
		 * enable/disable visualized timeout as progressbar under titlebar
		 * @param[in]	enable
		 * 	@li	expects type bool, default = true
		 */
		void enableTimeOutBar(bool enable = true){enable_timeout_bar = enable;}

		/**
		 * disable visualized timeout as progressbar under titlebar
		 * 	@see	enableTimeOutBar()
		 */
		void disableTimeOutBar(){enableTimeOutBar(false);}

		/**
		* init or unload visualized timeout as progressbar under titlebar
		* @param[in]	do_init
		* 	@li	type bool, default = true
		*/
		void initTimeOutBar(bool do_init = true);

		/**
		* unload visualized timeout as progressbar
		* 	@see initTimeOutBar
		*/
		void clearTimeOutBar(){initTimeOutBar(false);}

		/**
		* scroll handler for text objects: NOTE: exec() must be called !
		* @param[in]	hint_id
		* 	@li 	optional: expects type unsigned int, default = 0
		* 		default for the 1st hint item (=0), item id arises from the order of added items with addHintItem(), default we have minimal one item with id=0
		* @see		Scroll()
		*/
		void scroll_up(const uint& hint_id = 0);

		/**
		* scroll down handler for text objects: NOTE: exec() must be called !
		* @param[in]	hint_id
		* 	@li 	expects type unsigned int, default = 0
		* 		default for the 1st hint item (=0), item id arises from the order of added items with addHintItem(), default we h
		* @see		Scroll()
		*/
		void scroll_down(const uint& hint_id = 0);

		/**
		* Member to add a hint item
		* @param[in]	Text
		* 	@li 	expects type std::string, this is the message text inside the window, text is UTF-8 encoded
		* @param[in]	text_mode
		* 	@li 	optional: expects type int, defines the text modes for embedded text lines
		* 		Possible Modes defined in /gui/widget/textbox.h
		* 		AUTO_WIDTH
		* 		AUTO_HIGH
		* 		SCROLL
		* 		CENTER
		* 		RIGHT
		* 		TOP
		* 		BOTTOM
		* 		NO_AUTO_LINEBREAK
		* 		AUTO_LINEBREAK_NO_BREAKCHARS
		* @param[in]	Picon
		* 	@li 	optional: expects type std::string, defines the picon name on the left side of message text, default = NULL (non Icon)\n
		* 		special case: If picon == NEUTRINO_ICON_LOADER, then the animated loader icon known from CHourGlass object will be painted.
		* 	@see	CHourGlass()
		* @param[in]	color_text
		* 	@li 	optional: expects type fb_pixel_t, defines the text color, default = COL_MENUCONTENT_TEXT
		* * @param[in]	font_text
		* 	@li 	optional: expects type Font*, defines the text font type, default = NULL for system preset for message contents
		*/
		void addHintItem(	const std::string& Text,
					const int& text_mode = DEFAULT_HINTBOX_TEXT_MODE,
					const std::string& Picon = std::string(),
					const fb_pixel_t& color_text = COL_MENUCONTENT_TEXT,
					Font* font_text = NULL);

		/**
		* Member to add a hint item from specified cc-item type
		* @param[in]	cc_Item
		* 	@li 	expects type CComponentsItem*, allows to add any possible cc-item type
		* 
		* 	@see	/gui/components/cc_types.h
		*/
		void addHintItem(CComponentsItem* cc_Item){ccw_body->addCCItem(cc_Item);}

		/**
		* Sets a text to a hint item.
		* @param[in]	Text
		* 	@li 	expects type std::string, this is the message text inside the hint item, text is UTF-8 encoded
		* @param[in]	hint_id
		* 	@li 	optional: expects type unsigned int, default = 0 for the first or one and only item
		* @param[in]	text_mode
		* 	@li 	optional: expects type int, defines the text modes for embedded text lines
		* 		Possible Modes defined in /gui/widget/textbox.h
		* 		AUTO_WIDTH
		* 		AUTO_HIGH
		* 		SCROLL
		* 		CENTER
		* 		RIGHT
		* 		TOP
		* 		BOTTOM
		* 		NO_AUTO_LINEBREAK
		* 		AUTO_LINEBREAK_NO_BREAKCHARS
		* 		default: CTextBox::AUTO_WIDTH | CTextBox::AUTO_HIGH | CTextBox::CENTER
		* @param[in]	color_text
		* 	@li 	optional: expects type fb_pixel_t, defines the text color, default = COL_MENUCONTENT_TEXT
		* * @param[in]	style
		* 	@li 	optional: expects type int, defines the text style NOTE: only for dynamic font
		* 		possible styles are:
		* 		FONT_STYLE_REGULAR (default)
		*		FONT_STYLE_BOLD
		*		FONT_STYLE_ITALIC
		*/
		void setMsgText(const std::string& Text,
				const uint& hint_id = 0,
				const int& mode = CTextBox::AUTO_WIDTH | CTextBox::AUTO_HIGH | CTextBox::CENTER,
				Font* font_text = NULL,
				const fb_pixel_t& color_text = COL_MENUCONTENT_TEXT,
				const int& style = CComponentsText::FONT_STYLE_REGULAR);
		void setMsgText(const neutrino_locale_t& locale,
				const uint& hint_id = 0,
				const int& mode = CTextBox::AUTO_WIDTH | CTextBox::AUTO_HIGH | CTextBox::CENTER,
				Font* font_text = NULL,
				const fb_pixel_t& color_text = COL_MENUCONTENT_TEXT,
				const int& style = CComponentsText::FONT_STYLE_REGULAR);
};

/**
* Simplified methodes to show hintboxes on screen
* Text is UTF-8 encoded
* 	@see	for possible parameters take a look to CHintBox()
*/
int ShowHint(const neutrino_locale_t Caption, const char * const Text, const int Width = HINTBOX_MIN_WIDTH, int timeout = HINTBOX_DEFAULT_TIMEOUT, const char * const Icon = NULL,const char * const Picon = NULL, const int& header_buttons = 0);
int ShowHint(const neutrino_locale_t Caption, const neutrino_locale_t Text, const int Width = HINTBOX_MIN_WIDTH, int timeout = HINTBOX_DEFAULT_TIMEOUT, const char * const Icon = NULL, const char * const Picon = NULL, const int& header_buttons = 0);
int ShowHint(const char * const Caption, const char * const Text, const int Width = HINTBOX_MIN_WIDTH, int timeout = HINTBOX_DEFAULT_TIMEOUT, const char * const Icon = NULL, const char * const Picon = NULL, const int& header_buttons = 0);
int ShowHint(const char * const Caption, const neutrino_locale_t Text, const int Width = HINTBOX_MIN_WIDTH, int timeout = HINTBOX_DEFAULT_TIMEOUT, const char * const Icon = NULL, const char * const Picon = NULL, const int& header_buttons = 0);



//! Sub class of CHintBox. Shows a simplified hint as a text hint without header and footer.
/*!
CHint provides a text without header and footer,
optional disable/enable background
*/

class CHint : public CHintBox
{
	private:
		int delay;

		void initHint(bool enable_bg)
		{
			paint_bg = enable_bg;
			ccw_show_header = false;
			ccw_show_footer = false;
			cc_item_type.name = "wg.hint";
			delay = 0;
		}
	public:
		/**CHint Constructor
		* @param[in]	Text
		* 	@li 	expects type const char*, this is the message text inside the window, text is UTF-8 encoded
		* @param[in]	show_background
		* 	@li 	optional: expects type bool, enable/disable backround paint, default = true
		*/
		CHint(const char * const Text, bool show_background = true, const char * const Picon = NULL);
		/**CHint Constructor
		* @param[in]	Text
		* 	@li 	expects type neutrino_locale_t, this is the message text inside the window, text is UTF-8 encoded
		* @param[in]	show_background
		* 	@li 	optional: expects type bool, enable/disable backround paint, default = true
		*/
		CHint(const neutrino_locale_t Text, bool show_background = true, const char * const Picon = NULL);

		virtual void setDelay(int d) {delay = d;}

		virtual ~CHint()
		{
			if (delay)
			{
				setTimeOut(delay, false);
				exec();
			}
		};
};

//! Sub class of CHintBox. Shows a simplified hint as a text hint without footer and optional header.
/*!
CLoaderHint provides a small window with header and text item,
optional disable/enable background, picon and delay
*/

class CLoaderHint : public CHintBox
{
	private:
		int delay;

		void initHint(bool enable_bg, bool enable_header)
		{
			paint_bg = enable_bg;
			ccw_show_header = enable_header;
			ccw_show_footer = false;
			cc_item_type.name = "wg.loader_hint";
			delay = 1;
		}
	public:
		/**CLoaderHint Constructor
		* @param[in]	Text
		* 	@li 	expects type const char*, this is the message text inside the window, text is UTF-8 encoded
		* @param[in]	show_background
		* 	@li 	optional: expects type bool, enable/disable background paint, default = true
		* @param[in]	Picon
		* 	@li 	optional: expects type std::string, defines the picon name on the left side of message text, default = NEUTRINO_ICON_LOADER
		* @param[in]	enable_header
		* 	@li 	optional: expects type bool, enable or disable header for message window, default = false
		* 	@see	CHourGlass()
		*/
		CLoaderHint(const char * const Text, bool show_background = true, const char * const Picon = NEUTRINO_ICON_LOADER, bool enable_header = false);

		/**Overloaded localized CLoaderHint Constructor
		* @param[in]	Text
		* 	@li 	expects type neutrino_locale_t, this is the message text inside the window, text is UTF-8 encoded
		* 	@see	for other parameters: CLoaderHint()
		*/
		CLoaderHint(const neutrino_locale_t Text, bool show_background = true, const char * const Picon = NEUTRINO_ICON_LOADER, bool enable_header = false);

		virtual void setDelay(int d) {delay = d;}

		virtual ~CLoaderHint()
		{
			if (delay)
			{
				setTimeOut(delay, false);
				exec();
			}
		};
};

/**
* Simplified methodes to show hintboxes without titlebar and footer
* Text is UTF-8 encoded
* @param[in]	Text
* 	@li 	expects type neutrino_locale_t or const char*
* @param[in]	timeout
* 	@li 	optional: expects type int as seconds, default = HINTBOX_DEFAULT_TIMEOUT (get from settings)
* @param[in]	show_background
* 	@li 	optional: expects type bool, enable/disable backround paint, default = true
* 	@see	for possible text parameters take a look to CHintBox()
*/
int ShowHintS(const neutrino_locale_t Text, int timeout = HINTBOX_DEFAULT_TIMEOUT, bool show_background = true, const char * const Picon = NULL);
int ShowHintS(const char * const Text, int timeout = HINTBOX_DEFAULT_TIMEOUT, bool show_background = true, const char * const Picon = NULL);
int ShowHintS(const std::string &Text, int timeout = HINTBOX_DEFAULT_TIMEOUT, bool show_background = true, const char * const Picon = NULL);

#if 0
typedef struct hint_message_data_t
{
	sigc::slot<void> slot;
	std::string text;
	neutrino_locale_t text_locale;
	int timeout;
	bool show_background;
	const char *Picon;
// 	hint_message_data_t(): 	text(std::string()),
// 				text_locale(NONEXISTANT_LOCALE),
// 				timeout(HINTBOX_DEFAULT_TIMEOUT),
// 				show_background(true),
// 				Picon(NULL){}
} hint_message_data_struct_t;

/**
 * Simplified methodes to show hintboxes without titlebar and footer with mounted function as slot for custom action
 * Text is UTF-8 encoded
 * @param[in]	Text
 * 	@li 	expects type neutrino_locale_t or const char*
 * @param[in]	Slot
 * 	@li 	expects sigc::slot<void>
 * 	@li 	example:
 * 	@li 	sigc::slot<void> sl = sigc::mem_fun(g_Plugins, &CPlugins::loadPlugins);\n
 * 		ShowHintS(LOCALE_SERVICEMENU_GETPLUGINS_HINT, sl, 1);
 * 	@li 	or use a function with parameter(s):
 * 		sigc::slot<void> sl = sigc::bind(sigc::mem_fun(*this, &CMyClass::foo), arg1, arg2, arg3, arg4);\n
 * 		ShowHintS(LOCALE_SERVICEMENU_GETPLUGINS_HINT, sl, 1);
 * @param[in]	timeout
 * 	@li 	optional: expects type int as seconds, default = HINTBOX_DEFAULT_TIMEOUT (get from settings)
 * @param[in]	show_background
 * 	@li 	optional: expects type bool, enable/disable backround paint, default = true
 */
int ShowHintS(const neutrino_locale_t Text, const sigc::slot<void> &Slot, int timeout = HINTBOX_DEFAULT_TIMEOUT, bool show_background = true, const char * const Picon = NULL);
int ShowHintS(const char * const Text, const sigc::slot<void> &Slot, int timeout = HINTBOX_DEFAULT_TIMEOUT, bool show_background = true, const char * const Picon = NULL);
int ShowHintS(const std::string &Text, const sigc::slot<void> &Slot, int timeout = HINTBOX_DEFAULT_TIMEOUT, bool show_background = true, const char * const Picon = NULL);

int ShowHintS(const hint_message_data_t &hint_data);
int ShowHintS(const std::vector<hint_message_data_t> &v_hint_data);
#endif

#endif
