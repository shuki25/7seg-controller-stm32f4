/*
 * Filename: sync_gps_rtc.c
 * Created Date: Thursday, June 15th 2023, 2:41:20 pm
 * Author: Dr. Joshua Butler
 * 
 * Copyright (c) 2023 Joshua Butler
 */

#include <stdint.h>
#include <string.h>

#include "sync_gps_rtc.h"
#include "cmsis_os.h"

#define MAX_UART_RX_BUFFER_SIZE 256

static RTC_HandleTypeDef *__hrtc;
static UART_HandleTypeDef *__huart;
static TIM_HandleTypeDef *__htim;
static RingBuffer *__rx_ring_buffer;

static uint8_t __nmea_sentence_buffer[NMEA_MAX_LENGTH];

sync_gps_rtc_state_t sync_gps_rtc_init(RTC_HandleTypeDef *hrtc, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, RingBuffer *rx_ring_buffer)
{
    __hrtc = hrtc;
    __huart = huart;
    __htim = htim;
    __rx_ring_buffer = rx_ring_buffer;

    return SYNC_GPS_RTC_STATE_OK;
}

sync_gps_rtc_state_t sync_gps_rtc_sync(bcd_time_t *bcd)
{

    uint8_t is_sync_complete = 0;
    uint16_t timeout;
    uint8_t rx_value = 0;
    uint8_t i = 0;
    static uint32_t latency = 0;
    static uint8_t delay = 0;
    nmea_sentence_type_t nmea_sentence_type;
    nmea_status_t nmea_status = NMEA_ERROR;

    if (!__hrtc && !__huart) {
        return SYNC_GPS_RTC_STATE_ERROR;
    }
    if (__hrtc && __huart) {
        timeout = 3000; // 15 seconds

        while(!is_sync_complete) {
            
            memset(__nmea_sentence_buffer, 0, NMEA_MAX_LENGTH);
            i = 0;

            // Seek for initial '$' character
            while (rx_value != '$' && timeout != 0) {
                if (!is_ring_buffer_empty(__rx_ring_buffer)) {
                    ring_buffer_dequeue(__rx_ring_buffer, &rx_value);
                }
                osDelay(5);
                timeout--;
            }

            if (timeout == 0) {
                return SYNC_GPS_RTC_STATE_TIMEOUT;
            }

            __nmea_sentence_buffer[i++] = rx_value;

            // Read NMEA sentence
            while (rx_value != '\r' && timeout != 0) {
                if (!is_ring_buffer_empty(__rx_ring_buffer)) {
                    ring_buffer_dequeue(__rx_ring_buffer, &rx_value);
                    __nmea_sentence_buffer[i] = rx_value;
                    i++;
                }
                osDelay(5);
                timeout--;
            }
            __nmea_sentence_buffer[i] = '\0';

            if (timeout == 0) {
                return SYNC_GPS_RTC_STATE_TIMEOUT;
            }

            // Check if NMEA sentence is GPS RMC sentence
            nmea_sentence_type = nmea_get_sentence_type(__nmea_sentence_buffer);
            if (nmea_sentence_type == NMEA_GPRMC_SENTENCE) {
                latency = __HAL_TIM_GET_COUNTER(__htim);
                delay = (uint8_t)(latency / 10000) + 1;
                nmea_status = nmea_parse_rmc(bcd, __nmea_sentence_buffer, delay);
                if (nmea_status == NMEA_OK) {
                    is_sync_complete = 1;
                }
            }
        }
    }
    return SYNC_GPS_RTC_STATE_SYNCED;
}

void sync_gps_rtc_set(bcd_time_t *bcd)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    sTime.Hours = bcd->hours;
    sTime.Minutes = bcd->minutes;
    sTime.Seconds = bcd->seconds;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    if (HAL_RTC_SetTime(__hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }

    sDate.WeekDay = bcd->day_of_week;
    sDate.Month = bcd->month;
    sDate.Date = bcd->day;
    sDate.Year = (uint8_t)(bcd->year - (uint16_t)2000);

    if (HAL_RTC_SetDate(__hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }
}
