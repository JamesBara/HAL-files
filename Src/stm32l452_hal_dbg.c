#include "stm32l452_hal_dbg.h"

CoreDebug_Type  *core_dbg;
ITM_Type *itm;

void dbg_init()
{
    #ifdef DEBUG_SWV
        core_dbg->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        itm->TER |= ITM_TCR_ITMENA_Msk;
    #endif
    #if DEBUG_UART
        uart_conf(USART2, USART_SRC_HSI,PA3,PA2,115200,WORD_BITS_8,PARITY_NONE,STOP_BITS_1,OVERSAMPLING_8);
    #endif

}


/**
 * @brief Overwrite the weak function _write found in syscalls.c 
 *        to enable printf on nucleo stm32l452re.
 * 
 * @note to use printf usart2 needs to be enabled first on pins PA2 and PA3, stdio.h should be 
 *       included.
 * 
 */

int _write(int file, char *ptr, int len) {

	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
        #ifdef DEBUG_SWV
            ITM_SendChar(*ptr++);
        #endif
        #ifdef DEBUG_UART
            uart_transmit_byte(*ptr++);
        #endif
	}
	return len;
}

__attribute__((weak)) void error_handler(char* f,uint32_t l)
{
  #if (DEBUG_UART || DEBUG_SWV)

  if ((USART2->CR1 & USART_CR1_UE))
  {
    printf("Error on: %s in line: %ld",f,l);
  }
  #endif

    while (1)
    {

    }
}