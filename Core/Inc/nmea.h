/*
 * Filename: nmea.h
 * Created Date: Thursday, June 15th 2023, 1:52:43 pm
 * Author: Dr. Joshua Butler
 * 
 * Copyright (c) 2023 Joshua Butler
 * 
 * Last Modified: Thu Jun 15 2023
 * 
 * This header file reads and parses NMEA sentences from a GPS module.
 * 
 */

#ifndef __NMEA_H
#define __NMEA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bcd_util.h"

#define NMEA_GPRMC "$GPRMC"
#define NMEA_GPGGA "$GPGGA"

#define NMEA_MAX_LENGTH 82

typedef enum {
    NMEA_GPRMC_SENTENCE,
    NMEA_GPGGA_SENTENCE,
    NMEA_UNKNOWN_SENTENCE
} nmea_sentence_type_t;

typedef enum {
    NMEA_OK,
    NMEA_ERROR,
    NMEA_INVALID_SENTENCE
} nmea_status_t;

nmea_sentence_type_t nmea_get_sentence_type(uint8_t *sentence);
nmea_status_t nmea_parse_rmc(bcd_time_t *bcd, uint8_t *sentence, uint8_t delay);

#ifdef __cplusplus
}
#endif

#endif /* __NMEA_H */