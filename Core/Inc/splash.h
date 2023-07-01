/*
 * Filename: splash.h
 * Created Date: Friday, June 16th 2023, 3:21:21 pm
 * Author: Dr. Joshua Butler
 * 
 * Copyright (c) 2023 Joshua Butler
 */

#ifndef __SPLASH_H
#define __SPLASH_H

#include "main.h"
#include "seven_segment.h"
#include "ssd1306.h"

/**
 * @brief Displays a splash screen on the SSD1306 OLED display and the seven-segment display.
 * 
 * @param led Pointer to the seven-segment display structure.
 */
void splash(seven_segment_t *led);

#endif /* __SPLASH_H */