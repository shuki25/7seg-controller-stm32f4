/*
* bcd_util.c
* Created on: 2023-06-06
* Author: Joshua Butler, MD, MHI
*
* This file contains functions for converting between binary and BCD and date calculations
*/

#include "bcd_util.h"

uint8_t bcd_to_uint8(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

uint16_t bcd_to_uint16(uint16_t bcd) {
    return ((bcd >> 12) * 1000) + (((bcd >> 8) & 0x0F) * 100) + (((bcd >> 4) & 0x0F) * 10) + (bcd & 0x0F);
}

uint8_t uint8_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

uint16_t uint16_to_bcd(uint16_t dec) {
    return ((dec / 1000) << 12) | (((dec / 100) % 10) << 8) | (((dec / 10) % 10) << 4) | (dec % 10);
}

uint32_t bcd_days_in_month(uint8_t month, uint8_t year) {
    uint32_t days = 0;
    switch (month) {
        case 1:
            days = 31;
            break;
        case 2:
            days = 28;
            break;
        case 3:
            days = 31;
            break;
        case 4:
            days = 30;
            break;
        case 5:
            days = 31;
            break;
        case 6:
            days = 30;
            break;
        case 7:
            days = 31;
            break;
        case 8:
            days = 31;
            break;
        case 9:
            days = 30;
            break;
        case 10:
            days = 31;
            break;
        case 11:
            days = 30;
            break;
        case 12:
            days = 31;
            break;
        default:
            break;
    }
    if (month == 2) {
        if (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) {
            days = 29;
        }
    }
    return days;
}

int32_t bcd_days_between_dates(bcd_time_t *start, bcd_time_t *end) {
    int32_t days = 0;
    uint8_t month = start->month;
    uint16_t year = start->year;
    uint8_t day = start->day;
    uint8_t start_hour = start->hours;
    uint8_t start_minute = start->minutes;
    uint8_t start_second = start->seconds;
    uint8_t end_hour = end->hours;
    uint8_t end_minute = end->minutes;
    uint8_t end_second = end->seconds;

    // check if end date is before start date
    if (year > end->year) {
        return -1;
    } else if (year == end->year) {
        if (month > end->month) {
            return -1;
        } else if (month == end->month) {
            if (day > end->day) {
                return -1;
            }
        }
    }

    while (month != end->month || year != end->year || day != end->day) {
        days++;
        day++;
        if (day > bcd_days_in_month(month, year)) {
            day = 1;
            month++;
            if (month > 12) {
                month = 1;
                year++;
            }
        }
        
       
    }

    // Check if the difference is less than 24 hours
    if (days > 0) {
        if (start_hour > end_hour) {
            days--;
        } else if (start_hour == end_hour) {
            if (start_minute > end_minute) {
                days--;
            } else if (start_minute == end_minute) {
                if (start_second > end_second) {
                    days--;
                }
            }
        }
    }
    return days;
}

uint8_t bcd_hours_between_times(bcd_time_t *start, bcd_time_t *end) {
    uint8_t hours = 0;
    uint8_t start_hour = start->hours;
    uint8_t end_hour = end->hours;
    uint8_t start_minute = start->minutes;
    uint8_t end_minute = end->minutes;
    uint8_t start_second = start->seconds;
    uint8_t end_second = end->seconds;

    if (start_hour > end_hour) {
        hours = 24 - start_hour + end_hour;
    } else {
        hours = end_hour - start_hour;
    }
    if (start_minute > end_minute) {
        hours--;
    }
    if (hours == 0 && start_minute == end_minute && start_second > end_second) {
        hours = 23;
    }
    if (hours == 255) {
        hours = 23;
    }
    return hours;
}

uint8_t bcd_calculate_dow(bcd_time_t *date) {
    uint8_t dow = 0;
    uint8_t month = date->month;
    uint8_t year = date->year;
    uint8_t day = date->day;
    uint8_t century = year / 100;
    uint8_t year_of_century = year % 100;
    if (month < 3) {
        month += 12;
        year_of_century--;
    }
    dow = (day + (13 * (month + 1) / 5) + year_of_century + (year_of_century / 4) + (century / 4) + (5 * century)) % 7;
    return dow;
}

void bcd_rtc_to_bcd_time(RTC_TimeTypeDef *rtc_time, RTC_DateTypeDef *rtc_date, bcd_time_t *bcd_time) {
    bcd_time->seconds = rtc_time->Seconds;
    bcd_time->minutes = rtc_time->Minutes;
    bcd_time->hours = rtc_time->Hours;
    bcd_time->day = rtc_date->Date;
    bcd_time->month = rtc_date->Month;
    bcd_time->year = rtc_date->Year+2000;
    bcd_time->day_of_week = bcd_calculate_dow(bcd_time);
}
