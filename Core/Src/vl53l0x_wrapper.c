/*
 * vl5310x_wrapper.c
 *
 * Author: seojun LEE
 */

#include "vl53l0x_wrapper.h"
#include <stdio.h>

// 모드별 설정값
typedef struct {
    FixPoint1616_t SigmaLimit;
    FixPoint1616_t SignalLimit;
    uint32_t TimingBudget;
    uint8_t PreRangeVcselPeriod;
    uint8_t FinalRangeVcselPeriod;
} ModeConfig_t;

static const ModeConfig_t ModeConfigs[] = {
    // Default Mode
    {
        (FixPoint1616_t)(0.25 * 65536),
		(FixPoint1616_t)(0.1 * 65536), // 이 값을 바꾸면 최대 거리 변경
//        (FixPoint1616_t)(18 * 65536),
        33000,
        14,
        10
    },
    // High Precision Mode
    {
        (FixPoint1616_t)(0.25 * 65536),
        (FixPoint1616_t)(18 * 65536),
        200000,
        14,
        10
    },
    // Long Distance Mode
    {
        (FixPoint1616_t)(0.1 * 65536),
        (FixPoint1616_t)(60 * 65536),
        33000,
        18,
        14
    },
    // High Speed Mode
    {
        (FixPoint1616_t)(0.25 * 65536),
        (FixPoint1616_t)(32 * 65536),
        20000,
        14,
        10
    }
};

/**
 * VL53L0X 캘리브레이션
 */
static VL53L0X_Error VL53L0X_Calibration(VL53L0X_Dev_t *pDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;

    Status = VL53L0X_StaticInit(pDevice);
    if (Status != VL53L0X_ERROR_NONE) {
        return Status;
    }

    Status = VL53L0X_PerformRefCalibration(pDevice, &VhvSettings, &PhaseCal);
    if (Status != VL53L0X_ERROR_NONE) {
        return Status;
    }

    Status = VL53L0X_PerformRefSpadManagement(pDevice, &refSpadCount, &isApertureSpads);
    if (Status != VL53L0X_ERROR_NONE) {
        return Status;
    }

    return Status;
}

/**
 * 측정 모드 설정
 */
VL53L0X_Error VL53L0X_SetMode(VL53L0X_Dev_t *pDevice, uint8_t mode)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if (mode > MODE_HIGH_SPEED) {
        mode = MODE_DEFAULT;
    }

    const ModeConfig_t *config = &ModeConfigs[mode];

    Status = VL53L0X_StaticInit(pDevice);
    if (Status != VL53L0X_ERROR_NONE) return Status;

    Status = VL53L0X_PerformRefCalibration(pDevice, &VhvSettings, &PhaseCal);
    if (Status != VL53L0X_ERROR_NONE) return Status;
    HAL_Delay(2);

    Status = VL53L0X_SetLimitCheckEnable(pDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 0);
    if (Status != VL53L0X_ERROR_NONE) return Status;
    HAL_Delay(2);

    Status = VL53L0X_SetLimitCheckEnable(pDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if (Status != VL53L0X_ERROR_NONE) return Status;
    HAL_Delay(2);

    Status = VL53L0X_SetLimitCheckValue(pDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, config->SigmaLimit);
    if (Status != VL53L0X_ERROR_NONE) return Status;
    HAL_Delay(2);

    Status = VL53L0X_SetLimitCheckValue(pDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, config->SignalLimit);
    if (Status != VL53L0X_ERROR_NONE) return Status;
    HAL_Delay(2);

    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice, config->TimingBudget);
    if (Status != VL53L0X_ERROR_NONE) return Status;
    HAL_Delay(2);

    Status = VL53L0X_SetVcselPulsePeriod(pDevice, VL53L0X_VCSEL_PERIOD_PRE_RANGE, config->PreRangeVcselPeriod);
    if (Status != VL53L0X_ERROR_NONE) return Status;
    HAL_Delay(2);

    Status = VL53L0X_SetVcselPulsePeriod(pDevice, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, config->FinalRangeVcselPeriod);
    if (Status != VL53L0X_ERROR_NONE) return Status;

    return Status;
}

/**
 * VL53L0X 초기화
 */
VL53L0X_Error VL53L0X_DeviceInit(VL53L0X_Dev_t *pDevice, uint8_t mode)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    // 디바이스 파라미터 설정
    pDevice->I2cDevAddr = 0x52;        // 기본 I2C 주소
    pDevice->comms_type = 1;           // I2C 통신
    pDevice->comms_speed_khz = 100;    // 400kHz

    // 데이터 초기화
    Status = VL53L0X_DataInit(pDevice);
    if (Status != VL53L0X_ERROR_NONE) {
        return Status;
    }

    // 캘리브레이션
    Status = VL53L0X_Calibration(pDevice);
    if (Status != VL53L0X_ERROR_NONE) {
        return Status;
    }

    // 측정 모드 설정
    Status = VL53L0X_SetMode(pDevice, mode);
    if (Status != VL53L0X_ERROR_NONE) {
        return Status;
    }

    return Status;
}

/**
 * 단일 거리 측정
 */
VL53L0X_Error VL53L0X_SingleMeasurement(VL53L0X_Dev_t *pDevice, VL53L0X_RangingMeasurementData_t *pData)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    Status = VL53L0X_PerformSingleRangingMeasurement(pDevice, pData);
    if (Status != VL53L0X_ERROR_NONE) {
        return Status;
    }

    Status = VL53L0X_ClearInterruptMask(pDevice, 0);

    return Status;
}
