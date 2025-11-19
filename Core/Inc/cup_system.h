#ifndef CUP_SYSTEM_H
#define CUP_SYSTEM_H

#include <stdint.h>

extern uint8_t isIRTriggered;

void CupSystem_Init(void);
void CupSystem_Update(void);

#endif
