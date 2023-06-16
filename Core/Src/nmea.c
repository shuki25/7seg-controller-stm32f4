/*
 * Filename: nmea.c
 * Created Date: Thursday, June 15th 2023, 2:01:47 pm
 * Author: Dr. Joshua Butler
 * 
 * Copyright (c) 2023 Joshua Butler
 */

#include <string.h>
#include <stdlib.h>
#include "nmea.h"
#include "bcd_util.h"

/*
* This function returns index of the first character of the substring in the string.
*/
int substr_index(uint8_t *string, uint8_t *substring)
{
    int i = 0;
    int j = 0;
    int k = 0;
    int index = -1;

    while (string[i] != '\0')
    {
        if (string[i] == substring[j])
        {
            while (string[i + k] == substring[j + k])
            {
                if (substring[j + k + 1] == '\0')
                {
                    index = i;
                    break;
                }
                k++;
            }
        }
        i++;
    }
    return index;
}

int substr_index_next(uint8_t *string, uint8_t delimiter)
{
    int i = 0;
    int index = -1;

    while (string[i] != '\0')
    {
        if (string[i] == delimiter)
        {
            index = i;
            break;
        }
        i++;
    }
    return index;
}

/*
 * This function returns index of the first character of the string after n occurrences
 * of a separator character.
 */
int substr_index_after(uint8_t *string, uint8_t separator, __uint_least8_t n)
{
    int i = 0;
    int j = 0;
    int index = -1;

    while (string[i] != '\0')
    {
        if (string[i] == separator)
        {
            j++;
            if (j == n)
            {
                index = i + 1;
                break;
            }
        }
        i++;
    }
    return index;
}

/*
 * This function valdiates a NMEA sentence and returns the sentence type.
 */
nmea_sentence_type_t nmea_get_sentence_type(uint8_t *sentence)
{
    if (sentence[0] != '$')
    {
        return NMEA_UNKNOWN_SENTENCE;
    }
    if (substr_index(sentence, (uint8_t *)"GPRMC") != -1)
    {
        return NMEA_GPRMC_SENTENCE;
    }
    if (substr_index(sentence, (uint8_t *)"GPGGA") != -1)
    {
        return NMEA_GPGGA_SENTENCE;
    }
    return NMEA_UNKNOWN_SENTENCE;
}

/*
 * This function parses a GPRMC sentence and returns the time and date
 * in a bcd_time_t struct.
 *
 * Example GPRMC sentence:
 * $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68
 * $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68
 */
nmea_status_t nmea_parse_rmc(bcd_time_t *bcd, uint8_t *sentence, uint8_t delay)
{
    static char buf[NMEA_MAX_LENGTH];
    static char buf2[NMEA_MAX_LENGTH];
    static int i,j;

    if (sentence[0] != '$')
    {
        return NMEA_INVALID_SENTENCE;
    }
    if (substr_index(sentence, (uint8_t *)"GPRMC") == -1)
    {
        return NMEA_INVALID_SENTENCE;
    }

    i = substr_index_after(sentence, ',', 2);
    j = substr_index_next(sentence + i, ',') + i;
    memset(buf, 0, NMEA_MAX_LENGTH);
    strncpy(buf, (char *)sentence + i, j - i);

    if (buf[0] != 'A')
    {
        return NMEA_ERROR;
    }
    
    i = substr_index_after(sentence, ',', 1);
    j = substr_index_next(sentence + i, ',') + i;
    memset(buf, 0, NMEA_MAX_LENGTH);
    strncpy(buf, (char *)sentence + i, j - i);
    strncpy(buf2, buf, 2);
    bcd->hours = (uint8_t)(atoi(buf2));
    strncpy(buf2, buf + 2, 2);
    bcd->minutes = (uint8_t)(atoi(buf2));
    strncpy(buf2, buf + 4, 2);
    bcd->seconds = (uint8_t)(atoi(buf2)) + delay; // Add n seconds to account for GPS sync delay

    i = substr_index_after(sentence, ',', 9);
    j = substr_index_next(sentence + i, ',') + i;
    memset(buf, 0, NMEA_MAX_LENGTH);
    strncpy(buf, (char *)sentence + i, j - i);
    strncpy(buf2, buf, 2);
    bcd->day = (uint8_t)(atoi(buf2));
    strncpy(buf2, buf + 2, 2);
    bcd->month = (uint8_t)(atoi(buf2));
    strncpy(buf2, buf + 4, 2);
    bcd->year = (uint16_t)(atoi(buf2) + 2000);
    bcd->day_of_week = bcd_calculate_dow(bcd);

    return NMEA_OK;
}

