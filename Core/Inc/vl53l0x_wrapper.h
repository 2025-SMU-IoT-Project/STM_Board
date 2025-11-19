/*
 * vl5310x_wrapper.h
 *
 * Author: seojun LEE
 */

#ifndef VL53L0X_WRAPPER_H
#define VL53L0X_WRAPPER_H

#include "stm32f4xx_hal.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

extern I2C_HandleTypeDef hi2c1;

typedef enum {
    MODE_DEFAULT = 0,
    MODE_HIGH_PRECISION = 1,
    MODE_LONG_DISTANCE = 2,
    MODE_HIGH_SPEED = 3
} VL53L0X_Mode;

VL53L0X_Error VL53L0X_DeviceInit(VL53L0X_Dev_t *pDevice, uint8_t mode);
VL53L0X_Error VL53L0X_SingleMeasurement(VL53L0X_Dev_t *pDevice, VL53L0X_RangingMeasurementData_t *pData);
VL53L0X_Error VL53L0X_SetMode(VL53L0X_Dev_t *pDevice, uint8_t mode);

#endif /* VL53L0X_WRAPPER_H */
