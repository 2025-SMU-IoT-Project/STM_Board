/*
 * hx711.h
 *
 * Author:
 */

#ifndef __ULTRA_H_
#define __ULTRA_H_

#include "main.h"

void Ultra_Init(void);
float Ultra_GetDistance_CM(void);
float Ultra_CalibrateHeight(void);     //자동 보정 함수

#endif
