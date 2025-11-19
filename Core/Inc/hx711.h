/*
 * hx711.h
 *
 *  Created on: Nov 8, 2025
 *      Author: kimjim1n
 */

#ifndef __HX711_H__
#define __HX711_H__

#include "stm32f4xx_hal.h"

// 보통 128 GAIN, A채널만 씀.
#define HX711_GAIN_128  128
#define HX711_GAIN_64   64
#define HX711_GAIN_32   32

typedef struct {
    GPIO_TypeDef *dout_port;
    uint16_t      dout_pin;

    GPIO_TypeDef *sck_port;
    uint16_t      sck_pin;

    uint8_t       gain;     // 128 / 64 / 32

    int32_t       offset;   // 영점(offset) 값
    float         scale;    // (raw - offset) / scale = g(그램)
} HX711_t;

// 초기화: DWT 타이머 셋업 + 핀 정보 저장
void HX711_Init(HX711_t *hx,
                GPIO_TypeDef *dout_port, uint16_t dout_pin,
                GPIO_TypeDef *sck_port,  uint16_t sck_pin,
                uint8_t gain);

// raw 24bit 값 읽기
int32_t HX711_ReadRaw(HX711_t *hx);

// 영점 조절, 여러 번 읽어서 평균 & offset 저장/리턴
int32_t HX711_Tare(HX711_t *hx, uint8_t times);

int32_t HX711_Tare_Robust(HX711_t *hx, int n);

// 현재 무게(g) (scale 값 보정 이후 사용 가)
float HX711_GetWeight(HX711_t *hx, uint8_t times);

#endif /* __HX711_H__ */

