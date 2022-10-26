/**************************************************************************************
 *  How to use:
 *  
 *  The user can implement their own error_handler function. 
 * 
 *  @todo error_handler
 *  @todo modify printf() in the CMSIS core files to work over UART. 
 *  @todo Possibly get rid of the template system_stm32l4xx.c file and add the variables in this file:
 *  uint32_t SystemCoreClock = 4000000U;
 *  const uint8_t  AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
 *  const uint8_t  APBPrescTable[8] =  {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};
 *  const uint32_t MSIRangeTable[12] = {100000U,   200000U,   400000U,   800000U,  1000000U,  2000000U, \
 *                                     4000000U, 8000000U, 16000000U, 24000000U, 32000000U, 48000000U};
 * 
 ***************************************************************************************/
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