#ifndef STM32L452_HAL_SYS_H
#define STM32L452_HAL_SYS_H

#include "stdint.h"
#include "stm32l452xx.h"
#include "system_stm32l4xx.h"

#ifdef __cplusplus
extern "C" {
#endif


extern uint32_t hse_frequency;


void sys_set_systick(void); // Set systick in ms.
uint32_t sys_get_systick(void); //get the time in ms.
void error_handler(void);
void sys_ms_delay(uint32_t delay);



#ifdef __cplusplus
}
#endif
#endif //endif STM32L452_HAL_SYS_H