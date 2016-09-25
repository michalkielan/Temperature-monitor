#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

void SysTick_Configuration(void);

void mdelay(uint32_t ms);
void tick_delay(uint32_t tick);

#endif
