#include "stm32l452_hal_sys.h"

volatile uint32_t ms_tick_counter;
uint32_t hse_frequency;


void sys_set_systick(void)
{
    SysTick_Config(SystemCoreClock / 1000);
}



uint32_t sys_get_systick(void)
{
  return ms_tick_counter;
}

void sys_ms_delay(uint32_t delay)
{
    uint32_t start = sys_get_systick();

    while(sys_get_systick()-start<delay);
}




//TO-DO
__attribute__((weak)) void error_handler(void)
{
  while (1)
  {

  }

}

//Interrupt that increases the tick counter.
void SysTick_Handler(void)  {
  ms_tick_counter++;
}