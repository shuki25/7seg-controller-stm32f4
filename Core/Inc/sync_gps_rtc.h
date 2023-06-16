/*
 * Filename: sync_gps_rtc.h
 * Created Date: Thursday, June 15th 2023, 2:41:54 pm
 * Author: Dr. Joshua Butler
 * 
 * Copyright (c) 2023 Joshua Butler
 */

#ifndef __SYNC_GPS_RTC_H__
#define __SYNC_GPS_RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "ring_buffer.h"
#include "nmea.h"
#include "bcd_util.h"

typedef enum {
    SYNC_GPS_RTC_STATE_OK,
    SYNC_GPS_RTC_STATE_WAITING_FOR_SYNC,
    SYNC_GPS_RTC_STATE_SYNCED,
    SYNC_GPS_RTC_STATE_TIMEOUT,
    SYNC_GPS_RTC_STATE_ERROR
} sync_gps_rtc_state_t;

sync_gps_rtc_state_t sync_gps_rtc_init(RTC_HandleTypeDef *hrtc, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, RingBuffer *rx_ring_buffer);
sync_gps_rtc_state_t sync_gps_rtc_sync(bcd_time_t *bcd);
void sync_gps_rtc_set(bcd_time_t *bcd);

#ifdef __cplusplus
}
#endif

#endif // __SYNC_GPS_RTC_H__
