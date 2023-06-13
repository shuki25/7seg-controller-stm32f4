/*
* seven_segment.h
* Created on: 2023-06-03
* Author: Joshua Butler, MD, MHI
*
* This file contains the function prototypes for the seven segment display
* functions using WS2818B LED strip.
*
*/

#ifndef SEVEN_SEGMENT_H_
#define SEVEN_SEGMENT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "main.h"
#include "misc.h"

typedef enum uint8_t{
    SEVEN_SEGMENT_ERROR,
    SEVEN_SEGMENT_MALLOC_FAILED,
    SEVEN_SEGMENT_OK
} seven_segment_error_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint8_t counter_period;
    uint8_t duty_high;
    uint8_t duty_low;
    uint8_t data_sent_flag;
    uint8_t sacrificial_led_flag;
    uint8_t **data;
    uint8_t **mod;
    uint16_t *pwm_data;
} seven_segment_t;

seven_segment_error_t seven_segment_init(seven_segment_t *led_obj, TIM_HandleTypeDef *htim, const uint32_t channel, const uint8_t counter_period, uint8_t sacrificial_led_flag);
void seven_segment_set_LED(seven_segment_t *led_obj, uint8_t LEDnum, uint8_t Red, uint8_t Green, uint8_t Blue);
void seven_segment_set_brightness(seven_segment_t *led_obj, uint8_t brightness);
void seven_segment_set_digit(seven_segment_t *led_obj, uint8_t digit, uint8_t value, uint8_t color_group);
void seven_segment_set_blank(seven_segment_t *led_obj, uint8_t digit);
void seven_segment_WS2812_send(seven_segment_t *led_obj);

#ifdef __cplusplus
}
#endif
#endif /* SEVEN_SEGMENT_H_ */
