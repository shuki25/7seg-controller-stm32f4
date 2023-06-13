/*
* rtc_sync.h
*
*  Created on: May 24, 2023
*   Author: Joshua Butler
*
*/

#ifndef RTC_SYNC_H_
#define RTC_SYNC_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ring_buffer.h"

enum RTC_SYNC_STATUS {
    RTC_SYNC_STATUS_OK = 0,
    RTC_SYNC_STATUS_ERROR = 1,
    RTC_SYNC_STATUS_TIMEOUT = 2
};

void RTC_sync_init(RTC_HandleTypeDef *hrtc, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, RingBuffer *rx_ring_buffer);
uint8_t RTC_sync_start();
void RTC_sync_set(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);

#ifdef __cplusplus
}
#endif

#endif /* RTC_SYNC_H_ */