#ifndef STM32L452_DBG_H
#define STM32L452_DBG_H

#include "stm32l452xx.h"
#include "system_stm32l4xx.h"
#include <stdio.h>
#include <stdint.h>

    
#ifdef DEBUG_UART
    #include "stm32l452_hal_usart.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif


#if (DEBUG_UART || DEBUG_SWV)
void dbg_init();
#endif
void error_handler(char* f,uint32_t l);

//#define CoreDebug           ((CoreDebug_Type *)     CoreDebug_BASE)   /*!< Core Debug configuration struct */

#ifdef __cplusplus
}
#endif
#endif //endif STM32L452_USB_H