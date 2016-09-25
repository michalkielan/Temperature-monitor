#ifndef __UART_H
#define __UART_H

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "misc.h"


#define LED_BLUE 	GPIO_Pin_15
#define LED_ORANGE 	GPIO_Pin_13
#define LED_RED 	GPIO_Pin_14
#define LED_GREEN	GPIO_Pin_12
#define LED_GPIO 	GPIOD

volatile uint8_t rx_buf[20];
volatile uint8_t tx_buf[20];
volatile uint8_t frame_get_state;

void	uart_config(uint32_t baudrate);
uint8_t uart_free(void);
void 	uart_send_buf(unsigned char *buf, int len);

#endif
