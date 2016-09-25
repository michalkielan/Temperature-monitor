#include "UART.h"

/*extern global data*/
volatile uint8_t rx_buf[20],
 	 	 tx_buf[20],
 	 	 frame_get_state=0;

/* uart frame data*/
volatile uint8_t start_transmission=0,
		tx_head=0,
		rx_head=0,
		rx_len=0,
		*tx_ptr,	//pointer na bufor
		*rx_ptr;

void uart_config(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef  NVIC_InitStruct;

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);

    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

    /* Configure USART Tx as alternate function  */
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure USART Rx as alternate function  */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the USART1 Interrupt */
    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* USART configuration */
    USART_DeInit(USART1);
    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStruct);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* Enable USART */
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
	uint8_t rx_data=0;
	/*Rx*/
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		LED_GPIO->ODR ^= LED_ORANGE;
		rx_data = USART1->DR;

		if(rx_data=='$')
		{
			start_transmission=1;	/*zaczyna pobieraæ bajty*/
			rx_head=0;
			rx_buf[rx_head] = rx_data;
			rx_head++;
		}

		else
		{
			if(start_transmission)	/*jak pobiera bajty to..*/
			{
				rx_buf[rx_head]=rx_data;
				rx_head++;

				if(rx_data=='\n')
				{
					start_transmission=0;
					rx_len = rx_head;
					rx_head=0;
					frame_get_state=1;
				}

				if(rx_head>20)
				{
					start_transmission=0;
					rx_head=0;
				}
			}
		}

	}

	/*Tx*/
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		LED_GPIO->ODR ^= LED_RED;
		USART1->DR = tx_buf[tx_head++];
		if(tx_buf[tx_head-1] == '\n')
		{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
			tx_head=0;
		}
	}
}


