/*
* i2c_wrapper.c
* Created on: 2023-05-31
*  Author: Joshua Butler
*
* This file contains the wrapper functions for STM32 I2C driver.
*
*/

#include "i2c_wrapper.h"

int32_t i2c_read(void *handle, uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t size)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read((I2C_HandleTypeDef *)handle, (uint16_t)address<<1, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, buffer, size, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return -1;
    }
    return 0;
}

int32_t i2c_write(void *handle, uint8_t address, uint8_t reg, const uint8_t *buffer, uint16_t sizet)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write((I2C_HandleTypeDef *)handle, (uint16_t)address<<1, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buffer, sizet, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return -1;
    }
    return 0;
}
