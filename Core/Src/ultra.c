/*
 * ultra.c
 *
 * Author:
 */

#include "ultra.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;

static void DWT_DelayInit(void);
static void DWT_Delay_us(uint32_t us);
static void US_Trigger10us(void);
static int  US_MeasureEcho_us(uint32_t *us_out);

void Ultra_Init(void)
{
    DWT_DelayInit();
}

/* 10us 트리거 */
static void US_Trigger10us(void)
{
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    DWT_Delay_us(2);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    DWT_Delay_us(10);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

/* Echo 길이 측정 */
static int US_MeasureEcho_us(uint32_t *us_out)
{
    uint32_t timeout = HAL_GetTick() + 30;
    while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET)
        if (HAL_GetTick() > timeout)
        	return -1;

    uint32_t start = DWT->CYCCNT;

    timeout = HAL_GetTick() + 50;
    while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
        if (HAL_GetTick() > timeout)
        	return -1;

    uint32_t end = DWT->CYCCNT;

    *us_out = (end - start) / (SystemCoreClock / 1000000U);
    return 0;
}

/* cm로 거리 반환 */
float Ultra_GetDistance_CM(void)
{
    uint32_t echo_us = 0;
    US_Trigger10us();

    if(US_MeasureEcho_us(&echo_us) == 0)
    {
        float dist_cm = echo_us / 58.0f;
        return dist_cm;
    }
    return -1;
}

/* 자동 보정 (쓰레기통이 비었을 때 거리 측정) */
float Ultra_CalibrateHeight(void)
{
    float sum = 0;
    for(int i=0; i<10; i++)
    {
        float d = Ultra_GetDistance_CM();
        if(d <= 0) return -1;
        sum += d;
        HAL_Delay(100);
    }
    return sum / 10.0f;
}

/* DWT Delay 구현 */
static void DWT_DelayInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

static void DWT_Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000U) * us;
    while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}
