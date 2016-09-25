#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "misc.h"

uint16_t adc_buf[3];

void ADC_Initialize(void);

#endif
