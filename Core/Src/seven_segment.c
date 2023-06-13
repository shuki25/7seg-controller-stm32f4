/*
* seven_segment.c
* Created on: 2023-06-03
* Author: Joshua Butler, MD, MHI
*
* This file contains the functions for controlling the seven segment display.
*
*/

#include "seven_segment.h"
#include "misc.h"
#include <math.h>
#include <stdlib.h>

#define NUM_DIGITS 3
#define NUM_LED_DIGIT 29
#define USE_BRIGHTNESS 1
#define NUM_SACRIFICIAL_LED 1
#define PI 3.14159265358979323846

// #ifndef uint32_t
// typedef unsigned int uint32_t;
// #endif

const uint32_t digits[10] = {
		0b00011111111111101111111111110000, // 0
		0b00011110000000000000000011110000, // 1
		0b00000001111111100000111111111111, // 2
		0b00011111111000000000111111111111, // 3
		0b00011110000000001111000011111111, // 4
		0b00011111111000001111111100001111, // 5
		0b00011111111111101111111100001111, // 6
		0b00011110000000000000111111110000, // 7
		0b00011111111111101111111111111111, // 8
		0b00011111111000001111111111111111  // 9
};

const uint8_t color_groups[7][3] = {
		{255, 0, 0}, // red
		{0, 255, 0}, // green
		{0, 0, 255},  // blue
		{255, 255, 0}, // yellow
		{255, 0, 255}, // magenta
		{0, 255, 255}, // cyan
		{255, 255, 255} // white
};

seven_segment_error_t seven_segment_init(seven_segment_t *led_obj, TIM_HandleTypeDef *htim, const uint32_t channel, const uint8_t counter_period, uint8_t sacrificial_led_flag) {
    uint16_t num_leds = NUM_DIGITS * (NUM_LED_DIGIT + (NUM_SACRIFICIAL_LED * sacrificial_led_flag));

    led_obj->htim = htim;
    led_obj->channel = channel;
    led_obj->counter_period = counter_period;
    led_obj->duty_high = round(counter_period * 2/3);
    led_obj->duty_low = round(counter_period * 1/3);
    led_obj->data_sent_flag = 0;
    led_obj->sacrificial_led_flag = sacrificial_led_flag;
    led_obj->data = (uint8_t **)malloc(num_leds * sizeof(uint8_t *));
    if (led_obj->data == NULL) {
        return SEVEN_SEGMENT_MALLOC_FAILED;
    }
    for (int i=0; i<num_leds; i++) {
        led_obj->data[i] = (uint8_t *)malloc(4 * sizeof(uint8_t));
        if (led_obj->data[i] == NULL) {
            return SEVEN_SEGMENT_MALLOC_FAILED;
        }
    }
    for (int i=0; i<num_leds; i++) {
        led_obj->data[i][0] = i;
        led_obj->data[i][1] = 0;
        led_obj->data[i][2] = 0;
        led_obj->data[i][3] = 0;
    }
    led_obj->mod = (uint8_t **)malloc(num_leds * sizeof(uint8_t *));
    if (led_obj->mod == NULL) {
        return SEVEN_SEGMENT_MALLOC_FAILED;
    }
    for (int i=0; i<num_leds; i++) {
        led_obj->mod[i] = (uint8_t *)malloc(4 * sizeof(uint8_t));
        if (led_obj->mod[i] == NULL) {
            return SEVEN_SEGMENT_MALLOC_FAILED;
        }
    }
    led_obj->pwm_data = (uint16_t *)malloc((num_leds * 24) + 50);
    if (led_obj->pwm_data == NULL) {
        return SEVEN_SEGMENT_MALLOC_FAILED;
    }
    return SEVEN_SEGMENT_OK;
} 

void seven_segment_set_LED(seven_segment_t *led_obj, uint8_t LEDnum, uint8_t Red, uint8_t Green, uint8_t Blue) {
    led_obj->data[LEDnum][0] = LEDnum;
    led_obj->data[LEDnum][1] = Green;
    led_obj->data[LEDnum][2] = Red;
    led_obj->data[LEDnum][3] = Blue;
}

void seven_segment_set_brightness(seven_segment_t *led_obj, uint8_t brightness) {
#if USE_BRIGHTNESS
    if (brightness > 45) brightness = 45;
    for (int i=0; i< NUM_DIGITS * (NUM_LED_DIGIT+(led_obj->sacrificial_led_flag * NUM_SACRIFICIAL_LED)); i++) {
        led_obj->mod[i][0] = led_obj->data[i][0];
        for (int j=1; j<4; j++) {
            float angle = 90 - brightness;
            angle = angle*PI/180;
            led_obj->mod[i][j] = (led_obj->data[i][j])/(tan(angle));
        }
    }
#endif
}

void seven_segment_set_digit(seven_segment_t *led_obj, uint8_t digit, uint8_t value, uint8_t color_group) {
    uint8_t offset = NUM_LED_DIGIT * digit + (led_obj->sacrificial_led_flag * NUM_SACRIFICIAL_LED);
    for (int i=0; i<NUM_LED_DIGIT + led_obj->sacrificial_led_flag; i++) {
        uint8_t bit = (digits[value] >> i) & 0x1;
        if (bit) {
            seven_segment_set_LED(led_obj, offset + i, color_groups[color_group][0], color_groups[color_group][1], color_groups[color_group][2]);
        } else {
            seven_segment_set_LED(led_obj, offset + i, 0, 0, 0);
        }
    }
}

void seven_segment_set_blank(seven_segment_t *led_obj, uint8_t digit) {
    uint8_t offset = NUM_LED_DIGIT * digit + (led_obj->sacrificial_led_flag * NUM_SACRIFICIAL_LED);
    for (int i=0; i<NUM_LED_DIGIT + led_obj->sacrificial_led_flag; i++) {
        seven_segment_set_LED(led_obj, offset + i, 0, 0, 0);
    }
}

void seven_segment_WS2812_send(seven_segment_t *led_obj) {
    uint32_t indx=0;
    uint32_t color;

    for (int i= 0; i < (NUM_LED_DIGIT + (led_obj->sacrificial_led_flag * NUM_SACRIFICIAL_LED)) * NUM_DIGITS; i++) {
#if USE_BRIGHTNESS
        color = ((led_obj->mod[i][1]<<16) | (led_obj->mod[i][2]<<8) | (led_obj->mod[i][3]));
#else
        color = ((led_obj->data[i][1]<<16) | (led_obj->data[i][2]<<8) | (led_obj->data[i][3]));
#endif
    
            for (int i=23; i>=0; i--) {
                if (color&(1<<i)) {
                    led_obj->pwm_data[indx] = (uint16_t)led_obj->duty_high;  // 2/3 of counter_period
                }
                else led_obj->pwm_data[indx] = (uint16_t)led_obj->duty_low;  // 1/3 of counter_period
                indx++;
            }
        }
    
        for (int i=0; i<50; i++) {
            led_obj->pwm_data[indx] = 0;
            indx++;
        }
    
        HAL_TIM_PWM_Start_DMA(led_obj->htim, led_obj->channel, (uint32_t *)led_obj->pwm_data, indx);
        while (!led_obj->data_sent_flag){};
        led_obj->data_sent_flag = 0;
        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
    }
