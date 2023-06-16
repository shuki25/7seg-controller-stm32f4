/*
 * Filename: adj_date.h
 * Created Date: Thursday, June 15th 2023, 10:39:21 pm
 * Author: Dr. Joshua Butler
 * 
 * Copyright (c) 2023 Joshua Butler
 */

#ifndef __ADJ_DATE_H__
#define __ADJ_DATE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "bcd_util.h"
#include "ring_buffer.h"

typedef enum {
    ADJ_DATE_SAVE,
    ADJ_DATE_CANCEL,
    ADJ_DATE_TIMEOUT,
    ADJ_DATE_ERROR
} adj_date_status_t;

adj_date_status_t adj_date_task(bcd_time_t *bcd, bcd_time_t *current_bcd, RingBuffer *ring_buf);

#ifdef __cplusplus
} 
#endif
#endif // __ADJ_DATE_H__