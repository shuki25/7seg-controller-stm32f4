/*
* i2c_wrapper.h
*
* Created on: 2023-05-31
*  Author: Josh Butler
*/

#ifndef I2C_WRAPPER_H_
#define I2C_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"


int32_t i2c_read(void *handle, uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t size);
int32_t i2c_write(void *handle, uint8_t address, uint8_t reg, const uint8_t *buffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* I2C_WRAPPER_H_ */