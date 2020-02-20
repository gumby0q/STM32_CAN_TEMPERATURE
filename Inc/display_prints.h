#ifndef DISPLAY_PRINTS_H_
#define DISPLAY_PRINTS_H_


/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include "u8g2.h"

struct display_screen1_data
{
	char str_boiler_value[10];

	char str_tempreture_value_1[10];
	char str_humidity_value_1[10];

	char str_tempreture_value_2[10];
	char str_humidity_value_2[10];

	char str_pump_status_1[10];
	char str_pump_status_2[10];
};

void display_update2(u8g2_t *p_u8g2, struct display_screen1_data *screen_data);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* DISPLAY_PRINTS_H_ */
/** @}*/
