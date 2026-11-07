#ifndef __UI_H
#define __UI_H

#include "stm32f10x.h"
#include "path.h"

void UI_Init(void);
void UI_Update(void);
void UI_Tick5ms(void);
void UI_Announce(PathEvent event);
uint16_t UI_GetSpeedSet(void);

#endif
