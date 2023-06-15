/*
* eeprom.h
* Created on: 2023-06-13
* Author: Joshua Butler, MD, MHI
*
* Header file for eeprom.c
*/

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdint.h>

#define EEPROM_ADDRESS 0xA0
#define EEPROM_IDENTIFICATION 0xB0
#define EEPROM_PAGE_SIZE 32
#define EEPROM_PAGE_NUM 256

typedef enum {
    EEPROM_OK = 0,
    EEPROM_ERROR = 1,
    EEPROM_BUSY = 2,
    EEPROM_TIMEOUT = 3,
    EEPROM_MALLOC_FAILED = 4,
    EEPROM_WRITE_PROTECTED = 5
} eeprom_status_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    GPIO_TypeDef *wc_port;
    uint16_t wc_pin;
    uint8_t write_protected;
} eeprom_t;

eeprom_status_t eeprom_init(eeprom_t *eeprom, I2C_HandleTypeDef *hi2c, GPIO_TypeDef *wc_port, uint16_t wc_pin);
eeprom_status_t eeprom_write_protect(eeprom_t *eeprom, uint8_t state);
eeprom_status_t eeprom_write(eeprom_t *eeprom, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
eeprom_status_t eeprom_read(eeprom_t *eeprom, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* INC_EEPROM_H_ */