/*
 * Filename: splash.c
 * Created Date: Friday, June 16th 2023, 3:21:10 pm
 * Author: Dr. Joshua Butler
 * 
 * Copyright (c) 2023 Joshua Butler
 */

#include "splash.h"
#include "cmsis_os.h"

void splash(seven_segment_t *led)
{
    uint8_t i;
    ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(16, 3);
	ssd1306_WriteString("BUTLER", Font_16x26, White);
	ssd1306_UpdateScreen();

	seven_segment_set_blank(led, 0);
    seven_segment_set_blank(led, 1);
    seven_segment_set_blank(led, 2);

    for(i = 0; i < 11; i++) {
        seven_segment_set_digit(led, 0, i, 0);
        seven_segment_set_brightness(led, 25);
        seven_segment_WS2812_send(led);
        osDelay(125);
    }

    ssd1306_Fill(Black);
	ssd1306_SetCursor(3, 7);
	ssd1306_WriteString("ELECTRONICS", Font_11x18, White);
	ssd1306_UpdateScreen();

    for(i = 0; i < 12; i++) {
        seven_segment_set_digit(led, 1, i, 1);
        seven_segment_set_brightness(led, 25);
        seven_segment_WS2812_send(led);
        osDelay(125);
    }

    ssd1306_Fill(Black);
	ssd1306_SetCursor(20, 7);
	ssd1306_WriteString("(C) 2023", Font_11x18, White);
	ssd1306_UpdateScreen();

	for(i = 0; i < 13; i++) {
        seven_segment_set_digit(led, 2, i, 2);
        seven_segment_set_brightness(led, 25);
        seven_segment_WS2812_send(led);
        osDelay(125);
    }
}
