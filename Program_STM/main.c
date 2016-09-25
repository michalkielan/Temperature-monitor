/*C*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/*STM32*/
#include "stm32f4xx_tim.h"
/*USERS*/
#include "ADC.h"
#include "UART.h"
#include "DELAY.h"

/* defines */
#define DELAY 		450
#define ID			85

enum  {
	TEMP1_ADDRESS = 0,  /*adc dma addresses*/
	VOLTAGE_ADDRESS = 1,
	TEMP2_ADDRESS = 2,

	START_REQ = '0',   /*instructions*/
	START_RSP = '1',

	DATA_TEMP1_REQ = '2',
	DATA_VOLTAGE_REQ = '3',
	DATA_RSP = '4',
	DATA_TEMP2_REQ = '5',

	DEVICE_STATE_START = 0,   /*device state*/
	DEVICE_STATE_SEND_DATA,
	DEVICE_STATE_ERROR,

	SLAVE_START_STATE_OK = 0,	/*slave state*/
	SLAVE_START_STATE_ERROR,
	SLAVE_DATA_STATE_OK,
	SLAVE_DATA_STATE_ERROR,
	SLAVE_STATE_ADDRESS_ERROR,
	SLAVE_STATE_FRAME_ERROR
};


/* procedures */
void LED_Config(void);
void SendStartFrame(void);
void SendTemp1(void);
void SendTemp2(void);
void SendVoltage(void);
uint8_t GetSlaveState(void);
void Timer2_Init();

/*variable*/
volatile uint8_t  timeout_state=0;	//time variable
volatile uint32_t timeout_time=0;
volatile uint8_t  time_uart_send=0;

uint8_t device_data_state=0;	//state variable
uint8_t device_state=0;

uint8_t id[2];		//address id

int main()
{
	id[0] = ID / 10 + '0';
	id[1] = (ID % 10) + '0';

	LED_Config();
	uart_config(9600);
	ADC_Initialize();
	Timer2_Init();

	while (1)
	{
		switch (device_state)
		{
		case DEVICE_STATE_START:

			if (time_uart_send)
			{
				time_uart_send = 0;
				SendStartFrame();
			}

			if (frame_get_state)
			{
				frame_get_state = 0;

				if (GetSlaveState() == SLAVE_START_STATE_OK)
				{
					device_state = DEVICE_STATE_SEND_DATA;
				}

				else
				{
					device_data_state = DEVICE_STATE_ERROR;
				}
			}
			break;

		case DEVICE_STATE_SEND_DATA:

			if (time_uart_send)
			{
				if(device_data_state == 3)
				{
					device_data_state=0;
				}

				else
				{
					device_data_state++;
				}

				switch (device_data_state)
				{
				case 0:
					SendTemp1();
					break;

				case 1:
					SendTemp2();
					break;

				case 2:
					SendVoltage();
					break;

				default:
					break;
				}
			}

			if (frame_get_state)
			{
				frame_get_state = 0;

				if (GetSlaveState() != SLAVE_DATA_STATE_OK)
				{
					device_data_state = DEVICE_STATE_ERROR;
				}

			}
			break;

		case DEVICE_STATE_ERROR:

			/*reset buffor*/
			USART1->SR = 0;
			USART1->DR = 0;
			ADC1->DR = 0;

			/*reset data*/
			memset(&tx_buf, 0, 20);
			memset(&rx_buf, 0, 20);

			/*reset periph*/
			LED_Config();
			uart_config(9600);
			ADC_Initialize();
			Timer2_Init();

			device_state = DEVICE_STATE_START;

			break;

		default:
			break;
		}
		tick_delay(DELAY);
	}
}


void LED_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = LED_BLUE | LED_ORANGE | LED_RED | LED_GREEN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOD, &GPIO_InitStruct);
}


void SendStartFrame(void)
{
	uint8_t crc = 0, i = 0;
	uint8_t buf_iterator = 0;

	//memset(tx_buf,0,30);
	/*prepare buffor*/
	// tx_buf[buf_iterator++] = 0;
	tx_buf[buf_iterator++] = '$';
	tx_buf[buf_iterator++] = id[0]; //id
	tx_buf[buf_iterator++] = id[1];
	tx_buf[buf_iterator++] = ',';
	tx_buf[buf_iterator++] = START_REQ;
	tx_buf[buf_iterator++] = ',';
	tx_buf[buf_iterator++] = '0';
	tx_buf[buf_iterator++] = '0';
	tx_buf[buf_iterator++] = '0';
	tx_buf[buf_iterator++] = '*';

	for (i = 0; i < buf_iterator; i++)
	{
		crc += tx_buf[i];
	}

	tx_buf[buf_iterator++] = (crc / 100) + '0';
	tx_buf[buf_iterator++] = ((crc % 100) / 10) + '0';
	tx_buf[buf_iterator++] = (crc % 10) + '0';
	tx_buf[buf_iterator++] = '\r';
	tx_buf[buf_iterator++] = '\n';

	/*send UART data*/
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

}

void SendTemp1(void)
{
	uint8_t crc = 0, i = 0;
	uint8_t buf_iterator = 0;
	uint16_t TemperatureValue = 0;

	//memset(tx_buf,0,30);
	/*measure ADC*/
	for (i = 0; i < 40; i++)
	{
		TemperatureValue += adc_buf[TEMP1_ADDRESS];//adc_buf[TEMP_ADDRESS];
		tick_delay(1);
	}

	TemperatureValue /= 40;

	/*prepare buffor*/
	tx_buf[buf_iterator++] = '$';
	tx_buf[buf_iterator++] = id[0]; //id
	tx_buf[buf_iterator++] = id[1];
	tx_buf[buf_iterator++] = ',';
	tx_buf[buf_iterator++] = DATA_TEMP1_REQ;
	tx_buf[buf_iterator++] = ',';
	tx_buf[buf_iterator++] = (TemperatureValue / 100) + '0';
	tx_buf[buf_iterator++] = (TemperatureValue % 100) / 10 + '0';
	tx_buf[buf_iterator++] = (TemperatureValue % 10) + '0';
	tx_buf[buf_iterator++] = '*';

	for (i = 0; i < buf_iterator; i++)
	{
		crc += tx_buf[i];
	}

	tx_buf[buf_iterator++] = (crc / 100) + '0';
	tx_buf[buf_iterator++] = ((crc % 100) / 10) + '0';
	tx_buf[buf_iterator++] = (crc % 10) + '0';
	tx_buf[buf_iterator++] = '\r';
	tx_buf[buf_iterator++] = '\n';

	/*send UART data*/
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

}

void SendTemp2(void)
{
	uint8_t crc = 0, i = 0;
	uint8_t buf_iterator = 0;
	uint16_t TemperatureValue = 0;

	//memset(tx_buf,0,30);
	/*measure ADC*/
	for (i = 0; i < 40; i++)
	{
		TemperatureValue += adc_buf[TEMP2_ADDRESS];//adc_buf[TEMP_ADDRESS];
		tick_delay(1);
	}

	TemperatureValue /= 40;

	/*prepare buffor*/
	tx_buf[buf_iterator++] = '$';
	tx_buf[buf_iterator++] = id[0]; //id
	tx_buf[buf_iterator++] = id[1];
	tx_buf[buf_iterator++] = ',';
	tx_buf[buf_iterator++] = DATA_TEMP2_REQ;
	tx_buf[buf_iterator++] = ',';
	tx_buf[buf_iterator++] = (TemperatureValue / 100) + '0';
	tx_buf[buf_iterator++] = (TemperatureValue % 100) / 10 + '0';
	tx_buf[buf_iterator++] = (TemperatureValue % 10) + '0';
	tx_buf[buf_iterator++] = '*';

	for (i = 0; i < buf_iterator; i++)
	{
		crc += tx_buf[i];
	}

	tx_buf[buf_iterator++] = (crc / 100) + '0';
	tx_buf[buf_iterator++] = ((crc % 100) / 10) + '0';
	tx_buf[buf_iterator++] = (crc % 10) + '0';
	tx_buf[buf_iterator++] = '\r';
	tx_buf[buf_iterator++] = '\n';

	/*send UART data*/
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

}

void SendVoltage(void)
{
	uint8_t crc = 0, i = 0;
	uint8_t buf_iterator = 0;
	uint16_t VoltageValue = 0;

	//memset(tx_buf,0,30);
	/*measure ADC*/
	for (i = 0; i < 40; i++)
	{
		VoltageValue += adc_buf[VOLTAGE_ADDRESS];
	}

	VoltageValue /= 40;

	/*prepare buffor*/
	tx_buf[buf_iterator++] = '$';
	tx_buf[buf_iterator++] = id[0]; //id
	tx_buf[buf_iterator++] = id[1];
	tx_buf[buf_iterator++] = ',';
	tx_buf[buf_iterator++] = DATA_VOLTAGE_REQ;
	tx_buf[buf_iterator++] = ',';
	tx_buf[buf_iterator++] = (VoltageValue / 100) + '0';
	tx_buf[buf_iterator++] = (VoltageValue % 100) / 10 + '0';
	tx_buf[buf_iterator++] = (VoltageValue % 10) + '0';
	tx_buf[buf_iterator++] = '*';

	for (i = 0; i < buf_iterator; i++)
	{
		crc += tx_buf[i];
	}

	tx_buf[buf_iterator++] = (crc / 100) + '0';
	tx_buf[buf_iterator++] = ((crc % 100) / 10) + '0';
	tx_buf[buf_iterator++] = (crc % 10) + '0';
	tx_buf[buf_iterator++] = '\r';
	tx_buf[buf_iterator++] = '\n';

	/*send UART data*/
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

uint8_t GetSlaveState(void)
{
	uint8_t i = 0;

	uint8_t first_char = 0,
			address[2],
			instruction = 0,
			state[3],
			crc_frame = 0,
			crc_calculate = 0,
			crc_array[3];

 	  first_char = rx_buf[0];
 	  address[0] = rx_buf[1];
   	  address[1] = rx_buf[2];
	 instruction = rx_buf[4];
	    state[0] = rx_buf[6];
	    state[1] = rx_buf[7];
	    state[2] = rx_buf[8];
	crc_array[0] = rx_buf[10];
	crc_array[1] = rx_buf[11];
	crc_array[2] = rx_buf[12];

	crc_frame = atoi(crc_array);


	if (first_char == '$')
	{
		if (address[0] == id[0] && address[1] == id[1])
		{
			for (i = 0; i < 30; i++)
			{
				crc_calculate += rx_buf[i];

				if (rx_buf[i] == '*')
				{
					break;
				}
			}

			if (crc_calculate == crc_frame)
			{
				switch (instruction)
				{
				case START_RSP:
					if (state[2] == '1')
					{
						return SLAVE_START_STATE_OK;
					}

					else
					{
						return SLAVE_START_STATE_ERROR;
					}
					break;

				case DATA_RSP:
					if (state[2] == '1')
					{
						return SLAVE_DATA_STATE_OK;
					}

					else
					{
						return SLAVE_DATA_STATE_ERROR;
					}
					break;

				default:
					break;
				}
			}

			else
			{
				return SLAVE_STATE_FRAME_ERROR;
			}
		}

		else
		{
			return SLAVE_STATE_ADDRESS_ERROR;
		}

	}

	else
	{
		return SLAVE_STATE_FRAME_ERROR;
	}
}


GPIO_InitTypeDef  		GPIO_tim2;
TIM_TimeBaseInitTypeDef TIM_tim2;
NVIC_InitTypeDef 		NVIC_tim2;

void Timer2_Init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	//clock enable
	GPIO_tim2.GPIO_Pin   = GPIO_Pin_14;	        	//pins
	GPIO_tim2.GPIO_Mode  = GPIO_Mode_OUT;	 //OUT
	GPIO_tim2.GPIO_OType = GPIO_OType_PP;	 //push-pull
	GPIO_tim2.GPIO_Speed = GPIO_Speed_2MHz; //f 50Mhz
	GPIO_tim2.GPIO_PuPd  = GPIO_PuPd_NOPULL; //no pull-up
	GPIO_Init(GPIOD, &GPIO_tim2);		 //call-on GPIOE

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//72Mhz/preskaler/okres = f(int)
	TIM_tim2.TIM_Prescaler = 1200;
	TIM_tim2.TIM_Period = 1200;
	TIM_tim2.TIM_ClockDivision =0;
	TIM_tim2.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIM_tim2);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	NVIC_tim2.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_tim2.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_tim2.NVIC_IRQChannelSubPriority = 0;
	NVIC_tim2.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_tim2);
}

void TIM2_IRQHandler(void)
{
	time_uart_send = 1;

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		LED_GPIO->ODR ^= LED_BLUE;

		if (timeout_state)
		{
			timeout_time++;
		}
	}

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}





