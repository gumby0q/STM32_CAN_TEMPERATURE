#include "display_prints.h"
#include <stdio.h>


void display_update2(u8g2_t *p_u8g2, struct display_screen1_data *screen_data) {

	u8g2_SetFont(p_u8g2, u8g2_font_logisoso18_tr);

	char tmp_string[20];

	u8g2_FirstPage(p_u8g2);

	do {
		////// boiler temp
		sprintf(tmp_string, "B %s", screen_data->str_boiler_value);
		u8g2_DrawStr(p_u8g2, 0, 20, tmp_string);

		////// t1
		sprintf(tmp_string, "T %s", screen_data->str_tempreture_value_1);
		u8g2_DrawStr(p_u8g2, 0, 45, tmp_string);

		////// h1
		sprintf(tmp_string, "H %s", screen_data->str_humidity_value_2);
		u8g2_DrawStr(p_u8g2, 0, 70, tmp_string);

		////// t2
		sprintf(tmp_string, "T %s", screen_data->str_tempreture_value_1);
		u8g2_DrawStr(p_u8g2, 0, 100, tmp_string);

		////// h2
		sprintf(tmp_string, "H %s", screen_data->str_humidity_value_2);
		u8g2_DrawStr(p_u8g2, 0, 125, tmp_string);

//		////// h2
//		sprintf(tmp_string, "B %s", screen_data->str_boiler_value);
//		u8g2_DrawStr(p_u8g2, 0, 125, tmp_string);

	 } while (u8g2_NextPage(p_u8g2));
}
