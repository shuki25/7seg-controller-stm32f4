/*
 * Filename: adj_date.c
 * Created Date: Thursday, June 15th 2023, 10:40:43 pm
 * Author: Dr. Joshua Butler
 * 
 * Copyright (c) 2023 Joshua Butler
 * 
 * This file contains functions for adjusting the start date and time with IR remote.
 */

#include <string.h>
#include <stdio.h>
#include "adj_date.h"
#include "ssd1306.h"
#include "cmsis_os.h"

adj_date_status_t adj_date_task(bcd_time_t *bcd, bcd_time_t *current_bcd, RingBuffer *ring_buf)
{
    bcd_time_t orig_bcd;
    ring_buffer_flush(ring_buf);
    uint32_t remote_code = 0;
    uint8_t is_done = 0;
    char lcd_buffer[32];
    uint8_t blink = 0;
    uint8_t timeout = 100;

    memcpy(&orig_bcd, bcd, sizeof(bcd_time_t));

    ssd1306_Fill(Black);
    snprintf(lcd_buffer, sizeof(lcd_buffer), "Adj Date");
    ssd1306_SetCursor(20, 0);
    ssd1306_WriteString(lcd_buffer, Font_11x18, White);
    ssd1306_UpdateScreen();

    while(!is_done && timeout--) {
        remote_code = 0;

        if (!is_ring_buffer_empty(ring_buf)) {
            ring_buffer_dequeue(ring_buf, (void *)&remote_code);
            timeout = 100;
        }
        
        switch (remote_code) {
        case REMOTE_RIGHT:
            bcd->day++;
            if (bcd->day > bcd_days_in_month(bcd->month, bcd->year)) {
                bcd->day = 1;
                bcd->month++;
                if (bcd->month > 12) {
                    bcd->month = 1;
                    bcd->year++;
                }
            }
            break;
        case REMOTE_LEFT:
            if (bcd->day > 1) {
                bcd->day--;
            } else {
                bcd->month--;
                if (bcd->month < 1) {
                    bcd->month = 12;
                    bcd->year--;
                }
                bcd->day = bcd_days_in_month(bcd->month, bcd->year);
            }
            break;
        case REMOTE_UP:
            bcd->month++;
            if (bcd->month > 12) {
                bcd->month = 1;
                bcd->year++;
            }
            if (bcd->day > bcd_days_in_month(bcd->month, bcd->year)) {
                bcd->day = bcd_days_in_month(bcd->month, bcd->year);
            }
            break;
        case REMOTE_DOWN:
            if (bcd->month > 1) {
                bcd->month--;
            } else {
                bcd->month = 12;
                bcd->year--;
            }
            if (bcd->day > bcd_days_in_month(bcd->month, bcd->year)) {
                bcd->day = bcd_days_in_month(bcd->month, bcd->year);
            }
            break;
        case REMOTE_OK: // Confirm and save
            
            if (bcd_days_between_dates(bcd, current_bcd) >= 0) {
                is_done = 1;
                snprintf(lcd_buffer, sizeof(lcd_buffer), "Saving Changes");
                ssd1306_SetCursor(15, 20);
                ssd1306_WriteString(lcd_buffer, Font_7x10, White);
                ssd1306_UpdateScreen();
                return ADJ_DATE_SAVE;
            } else {
                snprintf(lcd_buffer, sizeof(lcd_buffer), " Invalid Date ");
                ssd1306_SetCursor(15, 20);
                ssd1306_WriteString(lcd_buffer, Font_7x10, White);
                ssd1306_UpdateScreen();
                osDelay(1000);
            }
            break;
        case REMOTE_POUND: // Cancel
            is_done = 1;
            memcpy(bcd, &orig_bcd, sizeof(bcd_time_t));
            snprintf(lcd_buffer, sizeof(lcd_buffer), "     Canceled     ");
            ssd1306_SetCursor(1, 20);
            ssd1306_WriteString(lcd_buffer, Font_7x10, White);
            ssd1306_UpdateScreen();
            osDelay(1000);
            return ADJ_DATE_CANCEL;
            break;
        default:
            break;
        }

        if (blink) {
            snprintf(lcd_buffer, sizeof(lcd_buffer), ">%04d-%02d-%02d<", bcd->year, bcd->month, bcd->day);
        } else {
            snprintf(lcd_buffer, sizeof(lcd_buffer), " %04d-%02d-%02d ", bcd->year, bcd->month, bcd->day);
        }
        ssd1306_SetCursor(22, 20);
        ssd1306_WriteString(lcd_buffer, Font_7x10, White);
        ssd1306_UpdateScreen();
        blink = !blink;
        osDelay(300);

    }
    if (timeout == 0) {
        memcpy(bcd, &orig_bcd, sizeof(bcd_time_t));
        snprintf(lcd_buffer, sizeof(lcd_buffer), "Timeout No Change");
        ssd1306_SetCursor(1, 20);
        ssd1306_WriteString(lcd_buffer, Font_7x10, White);
        ssd1306_UpdateScreen();
        osDelay(1000);
        return ADJ_DATE_TIMEOUT;
    }
    return ADJ_DATE_ERROR;
}
