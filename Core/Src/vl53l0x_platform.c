/*
 * vl5310x_platform.c
 *
 * Author: seojun LEE
 */

#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

#define VL53L0X_I2C_TIMEOUT 100

/**
 * I2C Write 함수
 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    HAL_StatusTypeDef hal_status;

    uint8_t *buffer = (uint8_t *)malloc(count + 1);
    if (buffer == NULL) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    buffer[0] = index;
    memcpy(&buffer[1], pdata, count);

    hal_status = HAL_I2C_Master_Transmit(&hi2c1, Dev->I2cDevAddr, buffer, count + 1, VL53L0X_I2C_TIMEOUT);

    free(buffer);

    if (hal_status != HAL_OK) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

/**
 * I2C Read 함수
 */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    HAL_StatusTypeDef hal_status;

    hal_status = HAL_I2C_Mem_Read(&hi2c1, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, pdata, count, VL53L0X_I2C_TIMEOUT);

    if (hal_status != HAL_OK) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

/**
 * 단일 바이트 Write
 */
VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

/**
 * 단일 바이트 Read
 */
VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

/**
 * Word Write (16bit)
 */
VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t buffer[2];
    buffer[0] = (data >> 8) & 0xFF;
    buffer[1] = data & 0xFF;
    return VL53L0X_WriteMulti(Dev, index, buffer, 2);
}

/**
 * Word Read (16bit)
 */
VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    uint8_t buffer[2];
    VL53L0X_Error Status = VL53L0X_ReadMulti(Dev, index, buffer, 2);
    *data = ((uint16_t)buffer[0] << 8) | buffer[1];
    return Status;
}

/**
 * DWord Write (32bit)
 */
VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buffer[4];
    buffer[0] = (data >> 24) & 0xFF;
    buffer[1] = (data >> 16) & 0xFF;
    buffer[2] = (data >> 8) & 0xFF;
    buffer[3] = data & 0xFF;
    return VL53L0X_WriteMulti(Dev, index, buffer, 4);
}

/**
 * DWord Read (32bit)
 */
VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    uint8_t buffer[4];
    VL53L0X_Error Status = VL53L0X_ReadMulti(Dev, index, buffer, 4);
    *data = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) |
            ((uint32_t)buffer[2] << 8) | buffer[3];
    return Status;
}

/**
 * 폴링 딜레이
 */
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    HAL_Delay(1);
    return VL53L0X_ERROR_NONE;
}

/**
 * UpdateByte - 특정 비트만 업데이트
 */
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;

    Status = VL53L0X_RdByte(Dev, index, &data);
    if (Status != VL53L0X_ERROR_NONE) {
        return Status;
    }

    data = (data & AndData) | OrData;

    Status = VL53L0X_WrByte(Dev, index, data);

    return Status;
}
