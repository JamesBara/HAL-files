#ifndef STM32L452_USB_DESC_H
#define STM32L452_USB_DESC_H

#include "stm32l452_hal_sys.h"
#include "stm32l452xx.h"
#include "system_stm32l4xx.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t device_descriptor[0x12];
extern uint8_t configuration_descriptor[0x43];








#ifdef __cplusplus
}
#endif
#endif //endif STM32L452_USB_DESC_H