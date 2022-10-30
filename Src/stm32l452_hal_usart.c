#include "stm32l452_hal_usart.h"
static void usart_enable_clock(USART_TypeDef *usart,usart_clk_src clk_src);
static uint8_t uart_gpio_init(USART_TypeDef *usart, gpio_pin rx, gpio_pin tx);

/**
 * @brief Enable the usart clock.
 * 
 * @param usart This parameter allows the choice between different USART peripherals and can be of value:
 *              USART1
 *              USART2
 *              USART3
 *              UART4
 * @param clk_src This parameter can be of value:
 *                USART_SRC_PCLK = 0x00, ///< Usart uses the peripheral clock (HCLK/ABP_prescaler).
 *                USART_SRC_SYSCLK = 0x01, ///< Usart uses the SYSCLK clock.
 *                USART_SRC_HSI = 0x02, ///< Usart uses HSI.
 *                USART_SRC_LSE = 0x03 ///< Usart uses LSE.
 */
static void usart_enable_clock(USART_TypeDef *usart,usart_clk_src clk_src)
{

    if (usart == USART1)
    {
        //Set the source
        RCC->CCIPR &= ~(RCC_CCIPR_USART1SEL);
        RCC->CCIPR |= clk_src<<RCC_CCIPR_USART1SEL_Pos;
        //Enable the usart clock
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;      
    }
    else if (usart == USART2)
    {
        //Set the source
        RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);
        RCC->CCIPR |= clk_src<<RCC_CCIPR_USART2SEL_Pos;
        //Enable the usart clock
        RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    }
    else if (usart == USART3)
    {
        //Set the source
        RCC->CCIPR &= ~(RCC_CCIPR_USART3SEL);
        RCC->CCIPR |= clk_src<<RCC_CCIPR_USART3SEL_Pos;
        //Enable the usart clock
        RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
    }
    else if (usart == UART4)
    {
        //Set the source
        RCC->CCIPR &= ~(RCC_CCIPR_UART4SEL);
        RCC->CCIPR |= clk_src<<RCC_CCIPR_UART4SEL_Pos;
        //Enable the usart clock
        RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
    }    
}

/**
 * @brief Initialize pins as alternative functions.
 * 
 * @note Any of alternative pins can be used. All pin are set with a pull-up resistor at very-high speed. 
 * 
 * @param usart This parameter allows the choice between different USART peripherals and can be of value:
 *              USART1
 *              USART2
 *              USART3
 *              UART4
 * @param rx This parameter is one of the rx pins of the selected uart.
 * @param tx This parameter is one of the tx pins of the selected uart.
 * @return uint8_t 0 if everything okay, 1 if error.
 */
static uint8_t uart_gpio_init(USART_TypeDef *usart, gpio_pin rx, gpio_pin tx)
{
    //Set the alternative funcion.
    gpio_alt_func alt_func_rx = AF7;
    gpio_alt_func alt_func_tx = AF7;
    //test if the gpios are valid for the selected usart.
    if (usart == USART1)
    {
        if ((rx!=PA10 && rx!=PB7) && (tx!=PA9 && tx!=PB6))
        {
            return 1;
        }
    }
    else if(usart == USART2)
    {
        if ((rx!=PA3 && rx!=PA15)&&(tx!=PA2))
        {
            return 1;
        }
        if (rx==PA15)
        {
            alt_func_rx = AF3;
        }
    }
    else if(usart == USART3)
    {
        if ((rx!=PC5 && rx!=PC11)&&(tx!=PC4 && tx!=PC10))
        {
            return 1;
        }
    }
    else if(usart == UART4)
    {
        if ((rx!=PA1 && rx!=PC11)&&(tx!=PA0 && tx!=PC10))
        {
            return 1;
        }
        alt_func_rx = AF8;
        alt_func_tx = AF8;
    }
    else
    {
        return 1;
    }   

    gpio_alt_func_init(tx, PP, SPEED_VERY_HIGH, PU, alt_func_tx);
    gpio_alt_func_init(rx, PP, SPEED_VERY_HIGH, PU, alt_func_rx);

    return 0;
}

/**
 * @brief Configure the uart.
 * 
 * @note This function only enable asynchronous blocking communication.
 * 
 * @todo Maybe add auto baud rate?
 * @todo add 9 bit word method.
 * 
 * @param usart This parameter allows the choice between different USART peripherals and can be of value:
 *              USART1
 *              USART2
 *              USART3
 *              UART4
 * @param clk_src This parameter can be of value:
 *                USART_SRC_PCLK = 0x00, ///< Usart uses the peripheral clock (HCLK/ABP_prescaler).
 *                USART_SRC_SYSCLK = 0x01, ///< Usart uses the SYSCLK clock.
 *                USART_SRC_HSI = 0x02, ///< Usart uses HSI.
 *                USART_SRC_LSE = 0x03 ///< Usart uses LSE.
 * @param rx This parameter is one of the rx pins of the selected uart.
 * @param tx This parameter is one of the tx pins of the selected uart.
 * @param baud_rate Common options of this parameter are the values:
 *                  2400
 *                  9600
 *                  19200
 *                  38400
 *                  57600
 *                  115200
 *                  230400
 *                  460800
 *                  921600
 *                  2000000
 *                  3000000                  
 * @param word_length This parameter can be of value:
 *                    WORD_BITS_8 = 0x0, ///<Default
 *                    WORD_BITS_9 = 0x1,
 *                    WORD_BITS_7 = 0x2
 * @param parity This parameter can be of value:
 *                    PARITY_NONE = 0x0, ///<Default
 *                    PARITY_EVEN = 0x1,
 *                    PARITY_ODD = 0x2
 * @param stop_bits This parameter can be of value:
 *                    STOP_BITS_1 = 0x0, ///<Default
 *                    STOP_BITS_0_5 = 0x1,
 *                    STOP_BITS_2 = 0x2,
 *                    STOP_BITS_1_5 = 0x3
 * @param oversampling This parameter can be of value:
 *                    OVERSAMPLING_16 = 0x0, ///<Default
 *                    OVERSAMPLING_8 = 0x1
 */
void uart_conf(USART_TypeDef *usart, usart_clk_src clk_src, gpio_pin rx, gpio_pin tx, uint32_t baud_rate, usart_word_length word_length, usart_parity parity, usart_stop_bits stop_bits, usart_oversampling oversampling)
{
    //Set the pins as alternative functions.
    if (uart_gpio_init(usart,rx,tx))
    {
        error_handler();
    }
    //Start the uart clock.
    usart_enable_clock(usart,clk_src);
    uint32_t usartdiv;
    apb_per_sel apb_per;
    //Select peripheral clock depending on usart.
    if (usart==USART1)
    {
        apb_per = APB_SEL_2;
    }
    else
    {
        apb_per = APB_SEL_1;
    }

    //Set the registers.
    //Disable uart.
    usart->CR1 &=  ~(USART_CR1_UE);

    //Calculate the baud rate.
    usartdiv = ((oversampling+1)*clock_get_periph_freq(apb_per))/baud_rate;
    //Set the baud rate.
    if (!oversampling)
    {
        usart->BRR = usartdiv;
    }
    else
    {
        usart->BRR &= ~(USART_BRR_DIV_MANTISSA);
        usart->BRR &= ~(USART_BRR_DIV_FRACTION);
        //Set bits [15:4]
        usart->BRR |= (usartdiv & USART_BRR_DIV_MANTISSA);
        //Set bits [2:0]
        usart->BRR |= ((usartdiv & USART_BRR_DIV_FRACTION)>>1);
        //Leave bit 3 cleared.
    }

    //Set the the number of bits.    
    usart->CR1 &= ~(USART_CR1_M1);
    usart->CR1 &= ~(USART_CR1_M0);
    usart->CR1 |= ((word_length>>1)<<USART_CR1_M1_Pos);
    usart->CR1 |= ((word_length & 0x01)<<USART_CR1_M0_Pos);
    //Set parity.
    usart->CR1 &= ~(USART_CR1_PCE);
    if (parity == PARITY_EVEN)
    {
        usart->CR1 |= USART_CR1_PCE;
        usart->CR1 &= ~(USART_CR1_PS);

    }
    else if (parity == PARITY_ODD)
    {
        usart->CR1 |= USART_CR1_PCE;
        usart->CR1 |= USART_CR1_PS;
    }
    //Set stop bits.
    usart->CR2 &= ~(USART_CR2_STOP);
    usart->CR2 |= (stop_bits<<USART_CR2_STOP_Pos);
    //Enable Receiver.
    usart->CR1 |= USART_CR1_RE;
    //Enable Transmiter
    usart->CR1 |= USART_CR1_TE;
    //Set oversampling.
    usart->CR1 &= ~(USART_CR1_OVER8);
    usart->CR1 |= (oversampling<<USART_CR1_OVER8_Pos);
    //Enable uart.
    usart->CR1 |=  USART_CR1_UE;
}

/**
 * @brief Receive data in blocking mode.
 * 
 * @note This function will fail if a 9 bit configuration with no parity is selected.
 * 
 * @param usart This parameter allows the choice between different USART peripherals and can be of value:
 *              USART1
 *              USART2
 *              USART3
 *              UART4
 * @param data This parameter corresponds to the buffer.
 * @param size This parameter is the amount of bytes to receive. Should be the exact amount of bytes to
 *             receive.
 * @param timeout This parameter corresponds to how long to wait for the Read Data Register to become empty.
 *                If it times out the function exits.                
 * @return uint8_t 0 if everything okay, 1 if error 
 */
uint8_t uart_received_data(USART_TypeDef *usart, uint8_t *data, uint32_t size, uint32_t timeout)
{
    uint16_t parity_mask = 0xFF;
    uint8_t word_length = 8;
    if (!data || !size)
    {
        return 1;
    }
    //Caclulate the parity mask and the word length.
    if ((usart->CR1 & USART_CR1_M1))
    {
        word_length = 7;
        parity_mask = 0x7F;
    }
    else
    {
        if ((usart->CR1 & USART_CR1_M0))
        {
            word_length = 9;
            parity_mask = 0x1FF;
        }
    }

    //In this case the data is 9 bits and no parity data doesn't fit uint8_t.
    if ((word_length==9) && !(usart->CR1 & USART_CR1_PCE))
    {
        return 1;
    }

    for (uint32_t i=0;i<size;i++)
    {

        uint32_t start = sys_get_systick();
        while (!(usart->ISR & USART_ISR_RXNE))
        {
                if (sys_get_systick()-start>timeout)
                {
                    return 1;
                }
        }
        *data = (usart->RDR & parity_mask);
        data++;        
    } 
    return 0;
}
/**
 * @brief Transmit data in blocking mode.
 * 
 * @note This function will fail if a 9 bit configuration with no parity is selected.
 * 
 * @param usart This parameter allows the choice between different USART peripherals and can be of value:
 *              USART1
 *              USART2
 *              USART3
 *              UART4
 * @param data This parameter corresponds to the buffer.
 * @param size This parameter is the amount of bytes to send. Should be the exact amount of bytes to
 *             be transmitted.
 * @param timeout This parameter corresponds to how long to wait for the Transmit Data Register to become empty.
 *                If it times out the function exits.                
 * @return uint8_t 0 if everything okay, 1 if error
 */
uint8_t uart_transmit_data(USART_TypeDef *usart, uint8_t *data, uint32_t size, uint32_t timeout)
{
    uint8_t word_length = 8;
    if (!data || !size)
    {
        return 1;
    }
    //Caclulate parity mask and the word length.
    if ((usart->CR1 & USART_CR1_M1))
    {
        word_length = 7;
    }
    else
    {
        if ((usart->CR1 & USART_CR1_M0))
        {
            word_length = 9;
        }
    }

    //In this case the data is 9 bits and no parity data doesn't fit uint8_t.
    if ((word_length==9) && !(usart->CR1 & USART_CR1_PCE))
    {
        return 1;
    }

    for (uint32_t i=0;i<size;i++)
    {

        uint32_t start = sys_get_systick();
        while (!(usart->ISR & USART_ISR_TXE))
        {
            if (sys_get_systick()-start>timeout)
            {
                return 1;
            }
        }
        usart->TDR = *data;
        data++;        
    } 
    return 0;
}

/**
 * @brief Overwrite the weak function _write found in syscalls.c 
 *        to enable printf on nucleo stm32l452re.
 * 
 * @note to use printf usart2 needs to be enabled first on pins PA2 and PA3, stdio.h should be 
 *       included.
 * 
 */
#ifdef DEBUG
int _write( int handle, char* data, int size ) {
    for (uint32_t i=0;i<size;i++)
    {

        uint32_t start = sys_get_systick();
        while (!(USART2->ISR & USART_ISR_TXE))
        {
            if (sys_get_systick()-start>500)
            {
                return -1;
            }
        }
        USART2->TDR = *data;
        data++;        
    } 
  return size;
}
#endif



/**
*@todo 
**/
void uart_it_conf(USART_TypeDef *usart, usart_clk_src clk_src, gpio_pin rx, gpio_pin tx, uint32_t baud_rate, usart_word_length word_length, usart_parity parity, usart_stop_bits stop_bits, usart_oversampling oversampling, idle_it idle_ie, uint32_t priority)
{
    IRQn_Type irq;
    uart_conf(usart, clk_src, rx, tx, baud_rate, word_length, parity, stop_bits, oversampling);
    //Enable Transmit data register, Transmission complete, Read data register and Parity error interrupts.
    usart->CR1 |= (USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE | USART_CR1_PEIE | (idle_ie<<USART_CR1_IDLEIE_Pos));
    //Enable framing error, overrun error or noise flag
    usart->CR3 |= USART_CR3_EIE;

    if (usart == USART1)
    {
        irq = USART1_IRQn;
    }
    else if(usart == USART2)
    {
        irq = USART2_IRQn;
    }
    else if(usart == USART3)
    {
        irq = USART3_IRQn;
    }
    else if(usart == UART4)
    {
        irq = UART4_IRQn;
    }
    else
    {
        error_handler();
    }

 	NVIC_SetPriority(irq,priority);
	NVIC_EnableIRQ(irq);
}

/************************************************
 *  Interrupt handlers.
 ***********************************************/
void USART1_IRQHandler()
{
	if (USART1->ISR & USART_ISR_TXE)
	{
        /*The user can clear the bit by implementing the callback function
         * and tranfering new data in TDR Register.
         */
        usart_it_txe_callback(USART1);
	}
    else if (USART1->ISR & USART_ISR_TC)
    {
        //Clear interrupt then handle it.
        USART1->ICR |= USART_ICR_TCCF;
        usart_it_tc_callback(USART1);
    }
    else if (USART1->ISR & USART_ISR_RXNE)
    {
        /*The user can clear the bit by implementing the callback function
         * and reading the data from the RDR Register.
         */
        usart_it_rxne_callback(USART1);
    }
    else if ((USART1->ISR & USART_ISR_PE) || (USART1->ISR & USART_CR3_EIE))
    {
        //The user should clear the bit of the error and handle it.
        usart_it_err_callback(USART1);
    }
    else if ((USART1->ISR & USART_ISR_IDLE))
    {
        //Clear interrupt then handle it.
        USART1->ICR |= USART_ICR_IDLECF;
        usart_it_idle_callback(USART1);
    }     
}

void USART2_IRQHandler()
{
	if (USART2->ISR & USART_ISR_TXE)
	{
        /*The user can clear the bit by implementing the callback function
         * and tranfering new data in TDR Register.
         */
        usart_it_txe_callback(USART2);
	}
    else if (USART2->ISR & USART_ISR_TC)
    {
        //Clear interrupt then handle it.
        USART2->ICR |= USART_ICR_TCCF;
        usart_it_tc_callback(USART2);
    }
    else if (USART2->ISR & USART_ISR_RXNE)
    {
        /*The user can clear the bit by implementing the callback function
         * and reading the data from the RDR Register.
         */
        usart_it_rxne_callback(USART2);
    }
    else if ((USART2->ISR & USART_ISR_PE) || (USART2->ISR & USART_CR3_EIE))
    {
        //The user should clear the bit of the error and handle it.
        usart_it_err_callback(USART2);
    }
    else if ((USART2->ISR & USART_ISR_IDLE))
    {
        //Clear interrupt then handle it.
        USART2->ICR |= USART_ICR_IDLECF;
        usart_it_idle_callback(USART2);
    }    
}

void USART3_IRQHandler()
{
	if (USART3->ISR & USART_ISR_TXE)
	{
        /*The user can clear the bit by implementing the callback function
         * and tranfering new data in TDR Register.
         */
        usart_it_txe_callback(USART3);
	}
    else if (USART3->ISR & USART_ISR_TC)
    {
        //Clear interrupt then handle it.
        USART3->ICR |= USART_ICR_TCCF;
        usart_it_tc_callback(USART3);
    }
    else if (USART3->ISR & USART_ISR_RXNE)
    {
        /*The user can clear the bit by implementing the callback function
         * and reading the data from the RDR Register.
         */
        usart_it_rxne_callback(USART3);
    }
    else if ((USART3->ISR & USART_ISR_PE) || (USART3->ISR & USART_CR3_EIE))
    {
        //The user should clear the bit of the error and handle it.
        usart_it_err_callback(USART3);
    }
    else if ((USART3->ISR & USART_ISR_IDLE))
    {
        //Clear interrupt then handle it.
        USART3->ICR |= USART_ICR_IDLECF;
        usart_it_idle_callback(USART3);
    }    
}

void UART4_IRQHandler()
{
	if (UART4->ISR & USART_ISR_TXE)
	{
        /*The user can clear the bit by implementing the callback function
         * and tranfering new data in TDR Register.
         */
        usart_it_txe_callback(UART4);
	}
    else if (UART4->ISR & USART_ISR_TC)
    {
        //Clear interrupt then handle it.
        UART4->ICR |= USART_ICR_TCCF;
        usart_it_tc_callback(UART4);
    }
    else if (UART4->ISR & USART_ISR_RXNE)
    {
        /*The user can clear the bit by implementing the callback function
         * and reading the data from the RDR Register.
         */
        usart_it_rxne_callback(UART4);
    }
    else if ((UART4->ISR & USART_ISR_PE) || (UART4->ISR & USART_CR3_EIE))
    {
        //The user should clear the bit of the error and handle it.
        usart_it_err_callback(UART4);
    }
    else if ((UART4->ISR & USART_ISR_IDLE))
    {
        //Clear interrupt then handle it.
        UART4->ICR |= USART_ICR_IDLECF;
        usart_it_idle_callback(UART4);
    }    
}

/**
 * @brief Callback function can be overwritten by the user.
 * 
 */
__attribute__((weak)) void usart_it_tc_callback(USART_TypeDef *usart)
{


}

__attribute__((weak)) void usart_it_txe_callback(USART_TypeDef *usart)
{


}

__attribute__((weak)) void usart_it_rxne_callback(USART_TypeDef *usart)
{


}
__attribute__((weak)) void usart_it_idle_callback(USART_TypeDef *usart)
{


}

__attribute__((weak)) void usart_it_err_callback(USART_TypeDef *usart)
{


}

/**
*@todo 
**/
void uart_dma_conf(USART_TypeDef *usart, usart_clk_src clk_src, gpio_pin rx, gpio_pin tx, uint32_t baud_rate, usart_word_length word_length, usart_parity parity, usart_stop_bits stop_bits, usart_oversampling oversampling)
{

}


/**
*@todo 
**/
void uart_hf_duplex_conf(USART_TypeDef *usart, usart_clk_src clk_src, gpio_pin rx, gpio_pin tx, uint32_t baud_rate, usart_word_length word_length, usart_parity parity, usart_stop_bits stop_bits, usart_oversampling oversampling)
{

}