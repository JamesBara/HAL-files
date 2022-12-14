#ifndef STM32L452_HAL_USART_H
#define STM32L452_HAL_USART_H

#include "stm32l452_hal_sys.h"
#include "stm32l452_hal_gpio.h"
#include "stm32l452_hal_clock.h"
#include "stm32l452_hal_dbg.h"

#ifdef __cplusplus
extern "C" {
#endif



typedef enum
{
    WORD_BITS_8 = 0x0,
    WORD_BITS_9 = 0x1,
    WORD_BITS_7 = 0x2
}usart_word_length;

typedef enum
{
    PARITY_NONE = 0x0,
    PARITY_EVEN = 0x1,
    PARITY_ODD = 0x2
}usart_parity;

typedef enum
{
    STOP_BITS_1 = 0x0,
    STOP_BITS_0_5 = 0x1,
    STOP_BITS_2 = 0x2,
    STOP_BITS_1_5 = 0x3
}usart_stop_bits;

typedef enum
{
    OVERSAMPLING_16 = 0x0,
    OVERSAMPLING_8 = 0x1
} usart_oversampling;


typedef enum
{
	USART_SRC_PCLK = 0x00, ///< Usart uses the peripheral clock (HCLK/ABP_prescaler).
	USART_SRC_SYSCLK = 0x01, ///< Usart uses the SYSCLK clock.
	USART_SRC_HSI = 0x02, ///< Usart uses HSI.
	USART_SRC_LSE = 0x03 ///< Usart uses LSE.
}usart_clk_src;

typedef enum
{
    IDLE_LINE_IT_DIS = 0x00, ///< Default
    IDLE_LINE_IT_EN = 0x01 ///< Enable interrupts for Idle line detected
}idle_it;

typedef enum
{
    CIRCULAR_MODE_DIS = 0x00,
    CIRCULAR_MODE_EN = 0x01
}circular_mode;


void uart_conf(USART_TypeDef *usart, usart_clk_src clk_src, gpio_pin rx, gpio_pin tx, uint32_t baud_rate, usart_word_length word_length, usart_parity parity, usart_stop_bits stop_bits, usart_oversampling oversampling);

uint8_t uart_received_data(USART_TypeDef *usart, uint8_t *data, uint32_t size, uint32_t timeout);
uint8_t uart_transmit_data(USART_TypeDef *usart, uint8_t *data, uint32_t size, uint32_t timeout);

void usart_it_tc_callback(USART_TypeDef *usart);
void usart_it_txe_callback(USART_TypeDef *usart);
void usart_it_rxne_callback(USART_TypeDef *usart);
void usart_it_idle_callback(USART_TypeDef *usart);
void usart_it_err_callback(USART_TypeDef *usart);

void uart_dma_transmit_conf(USART_TypeDef *usart, uint8_t *data, uint16_t size, circular_mode circ_mode, uint32_t priority);
void uart_dma_receive_conf(USART_TypeDef *usart, uint8_t *data, uint16_t size, circular_mode circ_mode, uint32_t priority);


void usart_dma_tx_hf_callback(USART_TypeDef *usart);
void usart_dma_tx_cmplt_callback(USART_TypeDef *usart);
void usart_dma_tx_err_callback(USART_TypeDef *usart);

void usart_dma_rx_hf_callback(USART_TypeDef *usart);
void usart_dma_rx_cmplt_callback(USART_TypeDef *usart);
void usart_dma_rx_err_callback(USART_TypeDef *usart);


#ifdef DEBUG_UART
uint8_t uart_transmit_byte(uint8_t ch);
#endif



#ifdef __cplusplus
}
#endif
#endif //endif STM32L452_HAL_USART_H