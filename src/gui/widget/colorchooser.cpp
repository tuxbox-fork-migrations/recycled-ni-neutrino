/*
	Based up Neutrino-GUI - Tuxbox-Project

	Copyright (C) 2001 Steffen Hehn 'McClean'
	Copyright (C) 2018 Thilo Graf

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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <gui/widget/colorchooser.h>

#include <global.h>
#include <neutrino.h>

#include <driver/fontrenderer.h>
#include <driver/rcinput.h>
#include <driver/screen_max.h>
#include <system/helpers.h>

#include <gui/color.h>
#include <gui/widget/msgbox.h>
#include <gui/widget/icons.h>

static const char *const icon_names[VALUES] =
{
	NEUTRINO_ICON_SLIDER_RED,
	NEUTRINO_ICON_SLIDER_GREEN,
	NEUTRINO_ICON_SLIDER_BLUE,
	NEUTRINO_ICON_SLIDER_ALPHA
};

static const neutrino_locale_t colorchooser_names[VALUES] =
{
	LOCALE_COLORCHOOSER_RED,
	LOCALE_COLORCHOOSER_GREEN,
	LOCALE_COLORCHOOSER_BLUE,
	LOCALE_COLORCHOOSER_ALPHA
};

CColorChooser::CColorChooser(const neutrino_locale_t Name, unsigned char *R, unsigned char *G, unsigned char *B, unsigned char *A, CChangeObserver *Observer)
{
	observer = Observer;
	name = Name;

	value[VALUE_R] = R;
	value[VALUE_G] = G;
	value[VALUE_B] = B;
	value[VALUE_A] = A;
}

void CColorChooser::Init()
{
	frameBuffer = CFrameBuffer::getInstance();
	header_height = g_Font[SNeutrinoSettings::FONT_TYPE_MENU_TITLE]->getHeight();
	font = g_Font[SNeutrinoSettings::FONT_TYPE_WINDOW_GENERAL];
	item_height = font->getHeight();

	// calculate max width of locals
	text_width = 0;
	for (int i = 0; i < VALUES; i++)
	{
		int tmp_text_width = font->getRenderWidth(g_Locale->getText(colorchooser_names[i]));
		if (tmp_text_width > text_width)
			text_width = tmp_text_width;
	}

	// assuming all sliders have same dimensions
	int dummy;
	frameBuffer->getIconSize(NEUTRINO_ICON_SLIDER_ALPHA, &slider_width, &dummy);

	bar_width = frameBuffer->scale2Res(200);
	/*
	   We have a half slider_width before and after the bar
	   to get the middle of the slider at the point of choise
	*/
	bar_offset = slider_width / 2;
	bar_full = bar_width + slider_width;

	preview_w = VALUES * item_height;
	preview_h = VALUES * item_height;

	width = w_max((text_width + bar_full + preview_w + 4 * OFFSET_INNER_MID), 0);
	height = h_max((header_height + VALUES * item_height + 2 * OFFSET_INNER_SMALL), 0);

	x = getScreenStartX(width);
	y = getScreenStartY(height);

	preview_x = x + text_width + bar_full + 3 * OFFSET_INNER_MID;
	preview_y = y + header_height + OFFSET_INNER_SMALL;

	chooser_gradient = gradient_none;
}

void CColorChooser::setColor()
{
	fb_pixel_t col = getColor();
	int y_prev = preview_y + ((preview_h - header_height) / 2);

	if ((g_settings.theme.menu_Head_gradient) && ((chooser_gradient == gradient_head_body) || (chooser_gradient == gradient_head_text)))
	{
		CComponentsHeader header(preview_x, y_prev, preview_w, header_height, "Head");
		if (chooser_gradient == gradient_head_body)
			header.setColorBody(col);
		else if (chooser_gradient == gradient_head_text)
			header.setCaptionColor(col);
		header.paint(CC_SAVE_SCREEN_NO);
	}
	else
	{
		CComponentsShapeSquare preview(preview_x, y_prev, preview_w, preview_h - header_height, NULL, false, COL_FRAME_PLUS_0, col);
		preview.setFrameThickness(FRAME_WIDTH_MIN);
		preview.setCorner(RADIUS_SMALL);
		preview.paint(false);
	}

	// paint color number
	int border_off = OFFSET_INNER_MIN + FRAME_WIDTH_MIN;

	char col_num[9];
	snprintf(col_num, sizeof(col_num), "%x", col);
	col_num[8] = '\0';

	int w_col_num = preview_w - 2 * border_off;
	int h_col_num = font->getHeight();

	Font *dfont = *CNeutrinoFonts::getInstance()->getDynFont(w_col_num, h_col_num, col_num);

	int x_col_num = preview_x + preview_w / 2 - dfont->getRenderWidth(col_num) / 2;
	printf("[CColorChooser] selected color = hex [%s] dec [%d]\n", col_num, col);
	PaintBoxRel(preview_x, preview_y, preview_w, h_col_num, COL_MENUCONTENT_PLUS_0);
	paintTextBoxRel(col_num, x_col_num, preview_y + border_off, w_col_num, h_col_num,
		dfont, CTextBox::AUTO_WIDTH | CTextBox::CENTER, CComponentsText::FONT_STYLE_REGULAR, COL_MENUCONTENT_TEXT, COL_MENUCONTENT_PLUS_0, CORNER_RADIUS_MIN, CORNER_ALL);
}

fb_pixel_t CColorChooser::getColor()
{
	int color = convertSetupColor2RGB(*(value[VALUE_R]), *(value[VALUE_G]), *(value[VALUE_B]));
	int alpha = (value[VALUE_A]) ? (convertSetupAlpha2Alpha(*(value[VALUE_A]))) : 0xFF;

	return (((alpha << 24) & 0xFF000000) | color);
}

int CColorChooser::exec(CMenuTarget *parent, const std::string &)
{
	Init();
	neutrino_msg_t msg;
	neutrino_msg_data_t data;

	int res = menu_return::RETURN_REPAINT;
	if (parent)
		parent->hide();

	unsigned char r_alt = *value[VALUE_R];
	unsigned char g_alt = *value[VALUE_G];
	unsigned char b_alt = *value[VALUE_B];
	unsigned char a_null = 0;
	unsigned char a_alt = (value[VALUE_A]) ? (*(value[VALUE_A])) : a_null;

	paint();
	setColor();

	int selected = 0;

	uint64_t timeoutEnd = CRCInput::calcTimeoutEnd(g_settings.timing[SNeutrinoSettings::TIMING_MENU]);

	bool loop = true;
	while (loop)
	{
		g_RCInput->getMsgAbsoluteTimeout(&msg, &data, &timeoutEnd, true);

		if (msg <= CRCInput::RC_MaxRC)
			timeoutEnd = CRCInput::calcTimeoutEnd(g_settings.timing[SNeutrinoSettings::TIMING_MENU]);

		int val = (*value[selected]);
		switch (msg)
		{
			case CRCInput::RC_down:
			{
				if (selected < ((value[VALUE_A]) ? 3 : 2))
				{
					paintSlider(x, y + header_height + OFFSET_INNER_SMALL + item_height * selected, value[selected], colorchooser_names[selected], icon_names[selected], false);
					selected++;
					paintSlider(x, y + header_height + OFFSET_INNER_SMALL + item_height * selected, value[selected], colorchooser_names[selected], icon_names[selected], true);
				}
				else
				{
					paintSlider(x, y + header_height + OFFSET_INNER_SMALL + item_height * selected, value[selected], colorchooser_names[selected], icon_names[selected], false);
					selected = 0;
					paintSlider(x, y + header_height + OFFSET_INNER_SMALL + item_height * selected, value[selected], colorchooser_names[selected], icon_names[selected], true);
				}
				break;

			}
			case CRCInput::RC_up:
			{
				if (selected > 0)
				{
					paintSlider(x, y + header_height + OFFSET_INNER_SMALL + item_height * selected, value[selected], colorchooser_names[selected], icon_names[selected], false);
					selected--;
					paintSlider(x, y + header_height + OFFSET_INNER_SMALL + item_height * selected, value[selected], colorchooser_names[selected], icon_names[selected], true);
				}
				else
				{
					paintSlider(x, y + header_height + OFFSET_INNER_SMALL + item_height * selected, value[selected], colorchooser_names[selected], icon_names[selected], false);
					selected = ((value[VALUE_A]) ? 3 : 2);
					paintSlider(x, y + header_height + OFFSET_INNER_SMALL + item_height * selected, value[selected], colorchooser_names[selected], icon_names[selected], true);
				}
				break;
			}
			case CRCInput::RC_right:
			{
				if (val < 100)
				{
					if (val < 98)
						val += 2;
					else
						val = 100;
					(*value[selected]) = (uint8_t)val;

					paintSlider(x, y + header_height + OFFSET_INNER_SMALL + item_height * selected, value[selected], colorchooser_names[selected], icon_names[selected], true);
					setColor();
				}
				break;
			}
			case CRCInput::RC_left:
			{
				if (val > 0)
				{
					if (val > 2)
						val -= 2;
					else
						val = 0;
					(*value[selected]) = (uint8_t)val;

					paintSlider(x, y + header_height + OFFSET_INNER_SMALL + item_height * selected, value[selected], colorchooser_names[selected], icon_names[selected], true);
					setColor();
				}
				break;
			}
			case CRCInput::RC_home:
			{
				if (((*value[VALUE_R] != r_alt) || (*value[VALUE_G] != g_alt) || (*value[VALUE_B] != b_alt) || ((value[VALUE_A]) && (*(value[VALUE_A]) != a_alt))) &&
					(ShowMsg(name, LOCALE_MESSAGEBOX_DISCARD, CMsgBox::mbrYes, CMsgBox::mbYes | CMsgBox::mbCancel) == CMsgBox::mbrCancel))
					break;

				*value[VALUE_R] = r_alt;
				*value[VALUE_G] = g_alt;
				*value[VALUE_B] = b_alt;
				if (value[VALUE_A])
					*value[VALUE_A] = a_alt;
				loop = false;
				break;
			}
			case CRCInput::RC_timeout:
			case CRCInput::RC_ok:
			{
				loop = false;
				break;
			}
			default:
			{
				if (CNeutrinoApp::getInstance()->listModeKey(msg))
				{
					break;
				}
				else if (CNeutrinoApp::getInstance()->handleMsg(msg, data) & messages_return::cancel_all)
				{
					loop = false;
					res = menu_return::RETURN_EXIT_ALL;
				}
			}
		}
	}

	hide();

	if (observer)
		observer->changeNotify(name, NULL);

	return res;
}

void CColorChooser::hide()
{
	frameBuffer->paintBackgroundBoxRel(x, y, width + OFFSET_SHADOW, height + header_height + OFFSET_SHADOW);
}

void CColorChooser::paint()
{
	CComponentsHeader header(x, y, width, header_height, g_Locale->getText(name), NEUTRINO_ICON_COLORS, CComponentsHeader::CC_BTN_EXIT);
	header.enableShadow(CC_SHADOW_RIGHT | CC_SHADOW_CORNER_TOP_RIGHT | CC_SHADOW_CORNER_BOTTOM_RIGHT);
	header.paint(CC_SAVE_SCREEN_NO);

	PaintBoxRel(x, y + header_height, width, height - header_height, COL_MENUCONTENT_PLUS_0, RADIUS_NONE, CORNER_NONE, CC_SHADOW_RIGHT | CC_SHADOW_CORNER_TOP_RIGHT | CC_SHADOW_CORNER_BOTTOM_RIGHT);

	for (int i = 0; i < VALUES; i++)
		paintSlider(x, y + header_height + OFFSET_INNER_SMALL + i * item_height, value[i], colorchooser_names[i], icon_names[i], (i == 0));

	CComponentsFooter footer(x, y + height, width, 0, CComponentsFooter::CC_BTN_LEFT | CComponentsFooter::CC_BTN_UP | CComponentsFooter::CC_BTN_DOWN | CComponentsFooter::CC_BTN_RIGHT);
	footer.setButtonLabel(NEUTRINO_ICON_BUTTON_OKAY, LOCALE_COLORSETUP_SAVE, width / 3);
	footer.enableShadow(CC_SHADOW_ON);
	footer.paint(CC_SAVE_SCREEN_NO);
}

void CColorChooser::paintSlider(int px, int py, unsigned char *spos, const neutrino_locale_t text, const char *const iconname, const bool selected)
{
	if (!spos)
		return;

	// clear area
	frameBuffer->paintBoxRel(px + text_width + 2 * OFFSET_INNER_MID, py, bar_full, item_height, COL_MENUCONTENT_PLUS_0);
	// paint bar
	/*
	   NEUTRINO_ICON_SLIDER_BODY should be scaled to bar_width.
	   So long we paint a simple frame. This is more save on higher resolutions.
	*/
	//frameBuffer->paintIcon(NEUTRINO_ICON_SLIDER_BODY, px + text_width + 2*OFFSET_INNER_MID + bar_offset, py, item_height);
	int w_col_rate = font->getRenderWidth("100");
	int w_bar = bar_width - w_col_rate;
	int x_bar = px + text_width + 2 * OFFSET_INNER_MID + bar_offset;
	frameBuffer->paintBoxFrame(x_bar, py + item_height / 3, w_bar, item_height / 3, 1, COL_FRAME_PLUS_0);

	// paint slider
	frameBuffer->paintIcon(selected ? iconname : NEUTRINO_ICON_SLIDER_INACTIVE, px + text_width + 2 * OFFSET_INNER_MID + ((*spos)*w_bar / 100), py, item_height);

	// paint color name
	paintTextBoxRel(g_Locale->getText(text), px + OFFSET_INNER_MID, py, text_width, item_height, font);
	// paint color rate
	paintTextBoxRel(to_string(*spos), x_bar + w_bar + 2 * OFFSET_INTER, py, w_col_rate, item_height, font, CTextBox::RIGHT);
}
