/*
	Based up Neutrino-GUI - Tuxbox-Project 
	Copyright (C) 2001 by Steffen Hehn 'McClean'

	Classes for generic GUI-related components.
	Copyright (C) 2012, 2013, Thilo Graf 'dbt'

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

#ifndef __CC_FORM_H__
#define __CC_FORM_H__


#include "config.h"
#include <gui/components/cc_base.h>
#include <gui/components/cc_item_text.h>
#include <gui/components/cc_item_picture.h>
#include <vector>
#include <string>


class CComponentsForm : public CComponentsItem
{
	private:
		void initVarForm();
	protected:
		std::vector<CComponentsItem*>	v_cc_items;			
		void paintForm(bool do_save_bg);
		///generates next possible index for an item, see also cc_item_index, getIndex(), setIndex()
		int genIndex();

		int append_h_offset;
		int append_v_offset;
	public:
		
		CComponentsForm();
		CComponentsForm(const int x_pos, const int y_pos, const int w, const int h, bool has_shadow = CC_SHADOW_OFF,
					fb_pixel_t color_frame = COL_MENUCONTENT_PLUS_6, fb_pixel_t color_body = COL_MENUCONTENT_PLUS_0, fb_pixel_t color_shadow = COL_MENUCONTENTDARK_PLUS_0);
		virtual ~CComponentsForm();
		
		void paint(bool do_save_bg = CC_SAVE_SCREEN_YES);
		void hide(bool no_restore = false);
		virtual void addCCItem(CComponentsItem* cc_Item);
		virtual void addCCItem(const std::vector<CComponentsItem*> &cc_items);
		virtual void insertCCItem(const uint& cc_item_id, CComponentsItem* cc_Item);
		virtual void removeCCItem(const uint& cc_item_id);
		virtual void removeCCItem(CComponentsItem* cc_Item);
		virtual void replaceCCItem(const uint& cc_item_id, CComponentsItem* new_cc_Item);
		virtual void replaceCCItem(CComponentsItem* old_cc_Item, CComponentsItem* new_cc_Item);
		virtual void exchangeCCItem(const uint& item_id_a, const uint& item_id_b);
		virtual void exchangeCCItem(CComponentsItem* item_a, CComponentsItem* item_b);
		virtual int getCCItemId(CComponentsItem* cc_Item);
		virtual CComponentsItem* getCCItem(const uint& cc_item_id);
		virtual void paintCCItems();
		virtual	void clearCCItems();
		virtual void cleanCCForm();
		virtual void setAppendOffset(const int &h_offset, const int& v_offset){append_h_offset = h_offset; append_v_offset = v_offset;};
};

class CComponentsIconForm : public CComponentsForm
{
	private:
		std::vector<std::string> v_icons;
		int ccif_offset, ccif_icon_align;
		void initMaxHeight(int *pheight);

	protected:
 		void initVarIconForm();

	public:
		CComponentsIconForm();
		CComponentsIconForm(const int x_pos, const int y_pos, const int w, const int h, const std::vector<std::string> &v_icon_names, bool has_shadow = CC_SHADOW_OFF,
					fb_pixel_t color_frame = COL_MENUCONTENT_PLUS_6, fb_pixel_t color_body = COL_MENUHEAD_PLUS_0, fb_pixel_t color_shadow = COL_MENUCONTENTDARK_PLUS_0);
// 		~CComponentsIconForm(); //inherited from CComponentsForm

		void paint(bool do_save_bg = CC_SAVE_SCREEN_YES);
		void initCCIcons();
		void addIcon(const std::string& icon_name);
		void addIcon(std::vector<std::string> icon_name);
		void removeIcons(){v_icons.clear();};
		void insertIcon(const uint& icon_id, const std::string& icon_name);
		void removeIcon(const uint& icon_id);
		void removeIcon(const std::string& icon_name);
		void removeAllIcons();
		void setIconOffset(const int offset){ccif_offset = offset;};

		enum //alignements
		{
			CC_ICONS_FRM_ALIGN_RIGHT ,
			CC_ICONS_FRM_ALIGN_LEFT
		};
		void setIconAlign(int alignment){ccif_icon_align = alignment;};

		int getIconId(const std::string& icon_name);
};



class CComponentsHeader : public CComponentsForm
{
	private:
		void initVarHeader();
	protected:
		CComponentsPicture * cch_icon_obj;
		CComponentsText * cch_text_obj;
		CComponentsIconForm * cch_btn_obj;
		std::string cch_text;
		const char*  cch_icon_name;
		fb_pixel_t cch_col_text;
		Font* cch_font;
		int cch_items_y, cch_icon_x, cch_icon_w, cch_text_x, cch_buttons, cch_buttons_w, cch_buttons_h, cch_buttons_space, cch_offset;
		std::vector<std::string> v_cch_btn;
		int cch_size_mode;
		int cch_caption_align;
		bool userHeight;

		void initIcon();
		void initCaption();
		void initButtons();
		void initDefaultButtons();
		void initButtonFormSize();

	public:
		enum
		{
			CC_BTN_HELP = 0x02,
			CC_BTN_INFO = 0x04,
			CC_BTN_MENU = 0x40,
			CC_BTN_EXIT = 0x80

		};

		enum
		{
			CC_HEADER_ITEM_ICON 	= 0,
			CC_HEADER_ITEM_TEXT 	= 1,
			CC_HEADER_ITEM_BUTTONS	= 2
		};

		enum
		{
			CC_HEADER_SIZE_LARGE 	= 0,
			CC_HEADER_SIZE_SMALL 	= 1
		};
		CComponentsHeader();
		CComponentsHeader(const int x_pos, const int y_pos, const int w, const int h = 0, const std::string& caption = "header", const char* icon_name = NULL, const int buttons = 0, bool has_shadow = CC_SHADOW_OFF,
					fb_pixel_t color_frame = COL_MENUCONTENT_PLUS_6, fb_pixel_t color_body = COL_MENUHEAD_PLUS_0, fb_pixel_t color_shadow = COL_MENUCONTENTDARK_PLUS_0);
		CComponentsHeader(const int x_pos, const int y_pos, const int w, const int h = 0, neutrino_locale_t caption_locale = NONEXISTANT_LOCALE, const char* icon_name = NULL, const int buttons = 0,bool has_shadow = CC_SHADOW_OFF,
					fb_pixel_t color_frame = COL_MENUCONTENT_PLUS_6, fb_pixel_t color_body = COL_MENUHEAD_PLUS_0, fb_pixel_t color_shadow = COL_MENUCONTENTDARK_PLUS_0);
		virtual ~CComponentsHeader();

		
		virtual void setCaption(const std::string& caption, const int& align_mode = CTextBox::NO_AUTO_LINEBREAK);
		virtual void setCaption(neutrino_locale_t caption_locale, const int& align_mode = CTextBox::NO_AUTO_LINEBREAK);
		virtual void setCaptionAlignment(const int& align_mode){cch_caption_align = align_mode;};
		virtual void setCaptionFont(Font* font_name);
		virtual void setCaptionColor(fb_pixel_t text_color){cch_col_text = text_color;};
		virtual void setOffset(const int offset){cch_offset = offset;};
		virtual void setIcon(const char* icon_name);
		virtual void addButtonIcon(const std::string& button_name);
		virtual void removeButtonIcons();
		virtual void setDefaultButtons(const int buttons);
		virtual void setButtonsSpace(const int buttons_space){cch_buttons_space = buttons_space;};
		virtual void initCCItems();
		virtual void setSizeMode(const int& size_mode){cch_size_mode = size_mode;};
		virtual CComponentsText* getTextObject(){return cch_text_obj;};
		virtual void paint(bool do_save_bg = CC_SAVE_SCREEN_YES);
};


class CComponentsFooter : public CComponentsHeader
{
	protected:
		void initVarFooter();
	public:
		CComponentsFooter();
		CComponentsFooter(	const int x_pos, const int y_pos, const int w, const int h = 0,
					const int buttons = 0,
					bool has_shadow = CC_SHADOW_OFF,
					fb_pixel_t color_frame = COL_MENUCONTENT_PLUS_6, fb_pixel_t color_body = COL_INFOBAR_SHADOW_PLUS_1, fb_pixel_t color_shadow = COL_MENUCONTENTDARK_PLUS_0);
};

//! Sub class of CComponentsForm. Shows a window with prepared items.
/*!
CComponentsWindow provides prepared items like header, footer and a container for
items like text, labels, pictures ...
*/
/*
        x
       y+-------------------------------------------------------+
        |icon        caption                            buttons |header (ccw_head)
        +-x-----------------------------------------------------+
        |cc_item0                                               |
        |cc_item1                                               |body (ccw_body)
        |               add items here directly with            |
        |               addWindowItem() or                      |
        y               with ccw_body->addCCItem()              |
        |               Note: x/y related to body object        |
        |                                                       |
        +-------------------------------------------------------+
        |        add cc_items with ccw_footer->addCCItem()      |footer(ccw_footer)
        +-------------------------------------------------------+

*/

class CComponentsWindow : public CComponentsForm
{
	protected:
		///object: header object, to get access to header properties see also getHeaderObject()
		CComponentsHeader * ccw_head;
		///object: body object, this is the container for all needed items, to add with addWindowItem()
		CComponentsForm * ccw_body;
		///object: footer object, to get access to header properties see also getFooterObject(
		CComponentsFooter * ccw_footer;
		///property: caption in header, see also getHeaderObject()
		std::string ccw_caption;
		///property: alignment mode for header caption
		int ccw_align_mode;
		///property: icon name in header, see also getHeaderObject()
		const char* ccw_icon_name;
		///property: assigned default icon buttons in header, see also getHeaderObject()
		int ccw_buttons;
		///property: value = true, let show footer, see showFooter()
		bool ccw_show_footer;
		///property: value = true, let show header, see showHeader()
		bool ccw_show_header;

		///initialze header object
		void initHeader();
		///initialze body object
		void initBody();
		///initialze footer object
		void initFooter();
		///initialze all window objects at once
		void initCCWItems();
		///initialize all attributes
		void initVarWindow();

	public:
		enum
		{
			CC_WINDOW_ITEM_HEADER 	= 0
		};
		///simple constructor for CComponentsWindow
		CComponentsWindow();
		
		///advanced constructor for CComponentsWindow, provides parameters for the most required properties, and caption as string
		CComponentsWindow(	const int x_pos, const int y_pos, const int w, const int h,
					const std::string& caption,
					const char* iconname = NULL,
					bool has_shadow = CC_SHADOW_OFF,
					fb_pixel_t color_frame = COL_MENUCONTENT_PLUS_6,
					fb_pixel_t color_body = COL_MENUCONTENT_PLUS_0,
					fb_pixel_t color_shadow = COL_MENUCONTENTDARK_PLUS_0);
		
		///advanced constructor for CComponentsWindow, provides parameters for the most required properties, and caption from locales
		CComponentsWindow(	const int x_pos, const int y_pos, const int w, const int h,
					neutrino_locale_t locale_caption,
					const char* iconname = NULL,
					bool has_shadow = CC_SHADOW_OFF,
					fb_pixel_t color_frame = COL_MENUCONTENT_PLUS_6,
					fb_pixel_t color_body = COL_MENUCONTENT_PLUS_0,
					fb_pixel_t color_shadow = COL_MENUCONTENTDARK_PLUS_0);

		///simple constructor for CComponentsWindow, provides parameters for caption as string and icon, position of window is general centered and bound
		///to current screen settings, this shows a window over full screen
		CComponentsWindow(const std::string& caption, const char* iconname = NULL);
		
		///simple constructor for CComponentsWindow, provides parameters for caption from locales and icon, position of window is general centered and bound
		///to current screen settings, this shows a window over full screen
		CComponentsWindow(neutrino_locale_t locale_caption, const char* iconname = NULL);
		
		~CComponentsWindow();

		///add item to body object, also usable is addCCItem() to add items to the windo object
		void addWindowItem(CComponentsItem* cc_Item);
		
		///allow/disallow paint a footer, default true, see also ccw_show_footer, showHeader()
		void showFooter(bool show = true){ccw_show_footer = show; initCCWItems();};
		///allow/disallow paint a header, default true, see also ccw_show_header, showFooter()
		void showHeader(bool show = true){ccw_show_header = show; initCCWItems();};

		///set caption in header with string, see also getHeaderObject()
		void setWindowCaption(const std::string& text, const int& align_mode = CTextBox::NO_AUTO_LINEBREAK){ccw_caption = text; ccw_align_mode = align_mode;};

		///set caption in header from locales, see also getHeaderObject()
		void setWindowCaption(neutrino_locale_t locale_text, const int& align_mode = CTextBox::NO_AUTO_LINEBREAK);
		///set caption alignment
		void setWindowCaptionAlignment(const int& align_mode){ccw_align_mode = align_mode;};

		///set icon name in header, see also getHeaderObject()
		void setWindowIcon(const char* iconname){ccw_icon_name = iconname;};

		///set default header icon buttons, see also getHeaderObject()
		void setWindowHeaderButtons(const int& buttons){ccw_buttons = buttons;};

		///returns a pointer to the internal header object, use this to get access to header properities
		CComponentsHeader* getHeaderObject(){return ccw_head;};
		
		///returns a pointer to the internal body object, use this to get access to body properities
		CComponentsForm* getBodyObject(){return ccw_body;};
		///returns a pointer to the internal footer object, use this to get access to footer properities
		CComponentsFooter* getFooterObject(){return ccw_footer;};

		///refresh position and dimension and reinitialize elemenatary properties
		void Refresh(){initCCWItems();};

		///paint all window items, this overwriting paint() from CComponentsForm
		virtual void paint(bool do_save_bg = CC_SAVE_SCREEN_YES);
};


class CComponentsExtTextForm : public CComponentsForm
{
	private:
		///property: content of label, see also setLabelAndText()
		std::string ccx_label_text;
		///property: content of text, see also setLabelAndText()
		std::string ccx_text;
		///property: color of label text, see also setLabelAndTextColor()
		fb_pixel_t ccx_label_color;
		///property: color of text, see also setLabelAndTextColor()
		fb_pixel_t ccx_text_color;
		///property: mode of label text, see also setTextModes()
		int ccx_label_align;
		///property: mode of text, see also setTextModes()
		int ccx_text_align;
		///property: width of label, see also setLabelWidthPercent()
		int ccx_label_width;
		///property: width of text, see also setLabelWidthPercent()
		int ccx_text_width;
		///property: font type of both items (label and text), see also setLabelAndText()
		Font* ccx_font;
		///property: percentage val of label width related to full width, causes fit of text automatically into the available remaining size of item, see also setLabelWidthPercent()
		uint8_t ccx_percent_label_w;
		
		///object: label object
		CComponentsLabel *ccx_label_obj;
		///object: text object
		CComponentsText *ccx_text_obj;
		
		///initialize of properties for all objects
		void initCCTextItems();
		///initialize the label object
		void initLabel();
		///initialize the text object
		void initText();

	protected:
		///initialize basic variables
		void initVarExtTextForm(const int& x_pos, const int& y_pos, const int& w, const int& h,
					const std::string& label_text, const std::string& text,
					bool has_shadow,
					fb_pixel_t label_color,
					fb_pixel_t text_color,
					fb_pixel_t color_frame, fb_pixel_t color_body, fb_pixel_t color_shadow);

	public:
		///advanced constructor for CComponentsExtTextForm, provides parameters for the most required properties, and caption as string
		CComponentsExtTextForm(	const int& x_pos = 1, const int& y_pos = 1, const int& w = 300, const int& h = 48,
					const std::string& label_text = "", const std::string& text = "",
					bool has_shadow = CC_SHADOW_OFF,
					fb_pixel_t label_color = COL_MENUCONTENTINACTIVE_TEXT,
					fb_pixel_t text_color = COL_MENUCONTENT_TEXT,
					fb_pixel_t color_frame = COL_MENUCONTENT_PLUS_6, fb_pixel_t color_body = COL_MENUCONTENT_PLUS_0, fb_pixel_t color_shadow = COL_MENUCONTENTDARK_PLUS_0);
// 		~CComponentsExtTextForm(); //inherited from CComponentsForm

		///assigns texts for label and text, parameter as string, parameter Font is optional for required font type, default font is dependently from defined item height
		void setLabelAndText(const std::string& label_text, const std::string& text,  Font* font_text = NULL);
		///assigns texts for label and text, parameter as neutrino_locale_t, parameter Font is optional for required font type, default font is dependently from defined item height
		void setLabelAndText(const neutrino_locale_t& locale_label_text, const neutrino_locale_t& locale_text,  Font* font_text = NULL);
		///assigns text Font type
		void setLabelAndTextFont(Font* font);
		
		///assigns texts for label and text, parameter as struct (locale_ext_txt_t), parameters provide the same properties like setLabelAndText()
		void setLabelAndTexts(const locale_ext_txt_t& texts);
		///assigns texts for label and text, parameter as struct (string_ext_txt_t), parameters provide the same properties like setLabelAndText()
		void setLabelAndTexts(const string_ext_txt_t& locale_texts);

		///assigns colors for text for label text, parameter as fb_pixel_t
		void setLabelAndTextColor(const fb_pixel_t label_color , const fb_pixel_t text_color);

		///assigns width of label and text related to width, parameter as uint8_t in percent of width, fits text automatically into the available remaining size of item
		void setLabelWidthPercent(const uint8_t& percent_val);

		///returns a pointer to the internal label object, use this to get access to its most properties
		CComponentsLabel*getLabelObject(){return ccx_label_obj;};
		///returns a pointer to the internal text object, use this to get access to its most properties
		CComponentsText*getTextObject(){return ccx_text_obj;};

		///sets the text modes (mainly text alignment) to the label and text object, see /gui/widget/textbox.h for possible modes
		void setTextModes(const int& label_mode, const int& text_mode);

		///paint this item/form
		void paint(bool do_save_bg = CC_SAVE_SCREEN_YES);
};

class CComponentsExtTextFormLocalized : public CComponentsExtTextForm
{
	public:
		///advanced constructor for CComponentsExtTextForm, provides parameters for the most required properties, and caption as locales
		CComponentsExtTextFormLocalized(const int& x_pos = 1, const int& y_pos = 1, const int& w = 300, const int& h = 48,
						const neutrino_locale_t& locale_label_text = NONEXISTANT_LOCALE, const neutrino_locale_t& locale_text = NONEXISTANT_LOCALE,
						bool has_shadow = CC_SHADOW_OFF,
						fb_pixel_t label_color = COL_MENUCONTENTINACTIVE_TEXT,
						fb_pixel_t text_color = COL_MENUCONTENT_TEXT,
						fb_pixel_t color_frame = COL_MENUCONTENT_PLUS_6, fb_pixel_t color_body = COL_MENUCONTENT_PLUS_0, fb_pixel_t color_shadow = COL_MENUCONTENTDARK_PLUS_0);
};

#endif
