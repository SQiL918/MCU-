#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f10x.h"

void Control_Init(void);
void Control_Reset(void);
void Control_Step(void);
float Control_GetLeftSpeed(void);
float Control_GetRightSpeed(void);
float Control_GetDistance(void);
uint8_t Control_GetGray(void);

#endif
