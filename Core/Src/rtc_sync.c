/*
 * rtc_sync.c
 *
 *  Created on: May 24, 2023
 *      Author: Joshua Butler
 */

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "rtc_sync.h"
#include "cmsis_os.h"

#define RTC_SYNC_UART_TIMEOUT 15000
#define MAX_RTC_SYNC_BUFFER_SIZE 64

RTC_HandleTypeDef *__hrtc;
UART_HandleTypeDef *__huart;
TIM_HandleTypeDef *__htim;
RingBuffer *__rx_ring_buffer;


uint8_t __uart_rx_buffer[MAX_RTC_SYNC_BUFFER_SIZE];
uint8_t __uart_tx_buffer[MAX_RTC_SYNC_BUFFER_SIZE];

void RTC_sync_init(RTC_HandleTypeDef *hrtc, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, RingBuffer *rx_ring_buffer)
{
    __hrtc = hrtc;
    __huart = huart;
    __htim = htim;
    __rx_ring_buffer = rx_ring_buffer;
}

uint8_t RTC_sync_start()
{
    HAL_StatusTypeDef status;

    uint8_t rx_value;
    uint16_t timeout = 0;
    int year, day, month, hour, minute, second, num_returned;
    uint16_t latency;
    char token[15];

    int is_sync_complete = 0;

    if (!__hrtc && !__huart) {
        return RTC_SYNC_STATUS_ERROR;
    }
    if (__hrtc && __huart) {
        
        __HAL_TIM_SET_COUNTER(__htim, 0);
        
        ring_buffer_flush(__rx_ring_buffer);
        sprintf((char *)__uart_tx_buffer, "sync\n");
        status = HAL_UART_Transmit(__huart, (uint8_t *)__uart_tx_buffer, strlen((char *)__uart_tx_buffer), RTC_SYNC_UART_TIMEOUT);
        if (status != HAL_OK) {
            if (status == HAL_TIMEOUT) {
                return RTC_SYNC_STATUS_TIMEOUT;
            }
            return RTC_SYNC_STATUS_ERROR;
        }

        while(!is_sync_complete) {
            int i = 0;
            timeout = 3000;
            rx_value = '\0';

            do {
                if (!is_ring_buffer_empty(__rx_ring_buffer)) {
                    ring_buffer_dequeue(__rx_ring_buffer, &rx_value);
                    __uart_rx_buffer[i++] = rx_value;
                }
                osDelay(5);
            } while (rx_value != '\n' && i < MAX_RTC_SYNC_BUFFER_SIZE && --timeout);

            if (timeout == 0 && rx_value != '\n') {
                return RTC_SYNC_STATUS_TIMEOUT;
            } else {
                __uart_rx_buffer[i] = '\0';
            }

            ring_buffer_flush(__rx_ring_buffer);

            num_returned = sscanf((char *)__uart_rx_buffer, "%s %d %d %d %d %d %d %d", token, (int *)&latency, &year, &month, &day, &hour, &minute, &second);
            if (num_returned < 8) {
                return RTC_SYNC_STATUS_ERROR;
            }
        
            if (strcmp(token, "datetime") == 0 && num_returned == 8) {
                // TODO: After UART signal, respond back with current time
                RTC_sync_set((uint8_t)year, (uint8_t)month, (uint8_t)day, (uint8_t)hour, (uint8_t)minute, (uint8_t)second);
                latency = __HAL_TIM_GET_COUNTER(__htim);
                sprintf((char *)__uart_tx_buffer, "ok %d %d %d %d %d %d %d\n", (int)latency, year, month, day, hour, minute, second);
                status = HAL_UART_Transmit(__huart, __uart_tx_buffer, strlen((char *)__uart_tx_buffer), RTC_SYNC_UART_TIMEOUT);
                if (status != HAL_OK) {
                    if (status == HAL_TIMEOUT) {
                        return RTC_SYNC_STATUS_TIMEOUT;
                    }
                    return RTC_SYNC_STATUS_ERROR;
                }
            } else if (strcmp(token, "ok") == 0 && num_returned == 8) {
                RTC_sync_set((uint8_t)year, (uint8_t)month, (uint8_t)day, (uint8_t)hour, (uint8_t)minute, (uint8_t)second);
                is_sync_complete = 1;
            }

        }
        return RTC_SYNC_STATUS_OK;
    }
    else {
        return RTC_SYNC_STATUS_ERROR;
    }
}

void RTC_sync_set(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    HAL_RTC_GetTime(__hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(__hrtc, &sDate, RTC_FORMAT_BIN);

    sTime.Seconds = second;
    sTime.Minutes = minute;
    sTime.Hours = hour;
    sDate.Date = day;
    sDate.Month = month;
    sDate.Year = year - 2000;

    HAL_RTC_SetTime(__hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(__hrtc, &sDate, RTC_FORMAT_BIN);
}
