#include "stm32f4xx.h"

static uint32_t TimingDelay;
extern uint32_t SystemCoreClock;

void SysTick_Configuration(void)
{
    if(SysTick_Config(SystemCoreClock / 1000)) {
        while(1);
    }
}

void mdelay(u32 ms)
{
    TimingDelay = ms;

    while(TimingDelay != 0);
}

void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00) {
        TimingDelay--;
    }
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    TimingDelay_Decrement();
}

void tick_delay(uint32_t tick)
{
  uint32_t delay=0;
  while(tick--)
  {
      for(delay=0;delay<0xFF;delay++);
  }

}
