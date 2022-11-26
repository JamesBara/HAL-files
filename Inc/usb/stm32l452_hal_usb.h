#ifndef STM32L452_USB_H
#define STM32L452_USB_H

#include "stm32l452_hal_sys.h"
#include "stm32l452xx.h"
#include "system_stm32l4xx.h"
#include "stm32l452_hal_gpio.h"
#include "stm32l452_hal_usb_desc.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif



/*
typedef enum 
{
    USB_CLASS_CDC = 0x00,
    USB_CLASS_AUDIO = 0x01,
    USB_CLASS_MSC = 0x02
}usb_class;
*/



#define GET_STATUS 0x00
/**
* @note For various reasons these cases
* are excluded. But may be implemented later.
* Mostly because I don't want to make a
* state machine.
*/
#if 0   
#define CLEAR_FEATURE 0x01
#define SET_FEATURE 0x03
#define SET_DESCRIPTOR 0x07
#define SET_CONFIGURATION 0x09
#define SET_INTERFACE 0x11
#define SET_ADDRESS 0x05
#endif 
#define GET_DESCRIPTOR 0x06
#define GET_CONFIGURATION 0x08
#define GET_INTERFACE 0x0A
#define SYNCH_FRAME 0x12

#define DESCRIPTOR_TYPE_DEVICE 0x01
#define DESCRIPTOR_TYPE_CONFIGURATION 0x02
#define DESCRIPTOR_TYPE_STRING 0x03
                        



/**
* @note According to USB 2.0 specification
* These cases are not used for standard calls.
*/
#if 0                
#define DESCRIPTOR_TYPE_INTERFACE 0x04
#define DESCRIPTOR_TYPE_ENDPOINT 0x05
#define DESCRIPTOR_TYPE_DEVICE_QUALIFIER 0x06
#define DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION 0x07
#define DESCRIPTOR_TYPE_INTERFACE_POWER1 0x08
#endif

//Change it to union to add double buffer.
typedef struct 
{
    uint16_t USB_ADDRn_TX;
    uint16_t USB_COUNTn_TX;
    uint16_t USB_ADDRn_RX;
    uint16_t USB_COUNTn_RX;
}btable __attribute__((aligned(4)));







void usb_init();
void usb_transmit(uint8_t *buffer, uint16_t size);

#ifdef __cplusplus
}
#endif
#endif //endif STM32L452_USB_H