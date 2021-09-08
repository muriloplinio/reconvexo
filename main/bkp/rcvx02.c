#include "rcvx02.h"

/**********************
 *       WIDGETS
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
static lv_style_t style0_button_1;

LV_IMG_DECLARE(default);


void rcvx02_create(lv_obj_t *parent)
{

#if LV_USE_BTN
	lv_style_copy(&style0_button_1, &lv_style_btn_rel);

	lv_obj_t *button_1 = lv_btn_create(parent, NULL);
	lv_obj_set_pos(button_1, 23, 19);
	lv_obj_set_size(button_1, 100, 35);
	lv_btn_set_toggle(button_1, false);
	lv_btn_set_style(button_1, LV_BTN_STYLE_REL, &style0_button_1);
#endif // LV_USE_BTN

}
