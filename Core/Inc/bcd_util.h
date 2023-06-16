/*
* bcd_util.h
* Created on: 2023-06-06
* Author: Joshua Butler, MD, MHI
*
* This file contains the function prototypes for the bcd_util.c file.
*/

#ifndef BCD_UTIL_H_
#define BCD_UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

// Stores date and time in binary coded decimal format
// Year is stored as a 4 digit number as uint16_t
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day_of_week;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} bcd_time_t;

uint8_t bcd_to_uint8(uint8_t bcd);
uint8_t uint8_to_bcd(uint8_t dec);
uint16_t bcd_to_uint16(uint16_t bcd);
uint16_t uint16_to_bcd(uint16_t dec);

uint32_t bcd_days_in_month(uint8_t month, uint8_t year);
int32_t bcd_days_between_dates(bcd_time_t *start, bcd_time_t *end);
uint8_t bcd_hours_between_times(bcd_time_t *start, bcd_time_t *end);
uint8_t bcd_calculate_dow(bcd_time_t *date);
void bcd_rtc_to_bcd_time(RTC_TimeTypeDef *rtc_time, RTC_DateTypeDef *rtc_date, bcd_time_t *bcd_time);

#ifdef __cplusplus
}
#endif

#endif /* BCD_UTIL_H_ */