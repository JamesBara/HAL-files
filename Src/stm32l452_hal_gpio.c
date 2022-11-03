/**************************************************************************************
 *  How to use:
 *  
 *  The user should first select the mode of their choice by calling the functions and 
 *  selecting the pin as shown on the pinout (ex. PA5):
 *  gpio_analog_conf(gpio_pin pin); ///< Analog mode.
 *  gpio_output_conf(gpio_pin pin, gpio_type type, gpio_speed speed,
 *  gpio_resistor resistor); ///< Output mode.
 *  gpio_input_conf(gpio_pin pin, gpio_resistor resistor); ///< Input mode.
 *  gpio_alt_func_init(gpio_pin pin, gpio_type type, gpio_speed speed,
 *  gpio_resistor resistor, gpio_alt_func alt_func); ///< Alternate function mode.
 *  
 *  Then the user can call the functions below to read, write or toggle the pin that they
 *  configured in the previous step in blocking:
 *  gpio_pin_state gpio_read(gpio_pin pin);
 *	gpio_write(gpio_pin pin, gpio_pin_state state);
 *  gpio_toggle(gpio_pin pin);
 * 
 * 	The user could also configure the use of an input with interrupts by calling the function:
 *  gpio_input_it_conf(gpio_pin pin, gpio_resistor resistor, gpio_edge_trigger edge, uint32_t priority);
 * 
 *  Then implement their own handler on the interrupt by using the function:
 *  void gpio_interrupt_callback(gpio_pin_num num);
 * 
 *  @todo Add grouping in interrupt priorities.
 *  @todo Disable clocks, interrupts and deinit the configurations.
 * 
 ***************************************************************************************/

#include "stm32l452_hal_gpio.h"

static void gpio_enable_clock(GPIO_TypeDef *gpio);
static void gpio_mode_select(GPIO_TypeDef *gpio, gpio_pin_num num, gpio_mode mode);
static GPIO_TypeDef *gpio_parse_gpio(gpio_pin pin);
static gpio_pin_num gpio_parse_pin_num(gpio_pin pin);

/**
 * @brief Enable the clock of the selected GPIO.
 * 
 * @param gpio Pointer to GPIO_TypeDef structure that contains
 * 				information about gpio related registers.
 */
static void gpio_enable_clock(GPIO_TypeDef *gpio)
{
		if (gpio == GPIOA)
		{
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
		}
		else if (gpio == GPIOB)
		{
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
		}
		else if (gpio == GPIOC)
		{
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
		}
		else if (gpio ==  GPIOH)
		{
			RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
		}
}

/**
 * @brief Select the mode of the selected GPIO and pin.
 * 
 * @param gpio Pointer to GPIO_TypeDef structure that contains
 * 				information about gpio related registers.
 * @param num This parameter can be any of the values GPIO_PIN_X where X= 0~15.
 * @param mode This parameter can be any of the values:
 *       		INPUT = 0x00, ///<Input mode
 *				OUTPUT = 0x01, ///< General purpose output mode
 *				ALT_FUNCTION = 0x02, ///< Alternate function mode
 *				ANALOG = 0x03 ///< Analog mode (reset state)
 */
static void gpio_mode_select(GPIO_TypeDef *gpio, gpio_pin_num num, gpio_mode mode)
{

		gpio->MODER &= ~(0x03 << num*0x02);
    	gpio->MODER |= (mode << num*0x02);
}

/**
 * @brief Parse the GPIO_TypeDef structure value from one of the gpio pins.
 * 
 * @param pin This parameter can be any of the values PXY (where X = A..C or H and Y = 0~15)
 * 				PXY is the friendly name of a pin as shown on the datasheet. 
 * @return GPIO_TypeDef*
 */
static GPIO_TypeDef *gpio_parse_gpio(gpio_pin pin)
{
	if ((pin>>0x04)==0x01)
	{
		return GPIOB;
	}
	else if ((pin>>0x04)==0x02)
	{
		return GPIOC;
	}
	else if ((pin>>0x04)==0x03)
	{
		return GPIOH;
	}
	else
	{
		return GPIOA;
	}
}
/**
 * @brief Parse the gpio_pin_num from one of the gpio pins.
 * 
 * @param pin This parameter can be any of the values PXY (where X = A..C or H and Y = 0~15)
 * 				PXY is the friendly name of a pin as shown on the datasheet. 
 * @return gpio_pin_num
 */
static gpio_pin_num gpio_parse_pin_num(gpio_pin pin)
{
	return (gpio_pin_num) (pin&0xF);
}

/**
 * @brief Set the selected pin in analog mode (reset).
 * 
 * @param pin This parameter can be any of the values PXY (where X = A..C or H and Y = 0~15)
 * 				PXY is the friendly name of a pin as shown on the datasheet. 
 */
void gpio_analog_conf(gpio_pin pin)
{
	GPIO_TypeDef *gpio = gpio_parse_gpio(pin);
	gpio_pin_num num = gpio_parse_pin_num(pin);
	gpio_enable_clock(gpio);
	gpio_mode_select(gpio,num,ANALOG);
}

/**
 * @brief Set the selected pin in output mode.
 * 
 * @param pin This parameter can be any of the values PXY (where X = A..C or H and Y = 0~15)
 * 				PXY is the friendly name of a pin as shown on the datasheet. 
 * @param type This parameter can be any of the values:
 * 				PP = 0x00, ///< Output push-pull (reset state)
 *				OD = 0x01 ///< Output open-drain
 * @param speed This parameter can be any of the values:
 *			 	SPEED_LOW = 0x00,
 *				SPEED_MEDIUM = 0x01,
 *				SPEED_HIGH = 0x02,
 *				SPEED_VERY_HIGH = 0x03
 * @param resistor This parameter can be any of the values:
 *					NO_PUPD = 0x00, ///< No pull-up, pull-down
 *					PU = 0x01, ///< Pull-up
 *					PD = 0x02 ///< Pull-down
 */
void gpio_output_conf(gpio_pin pin, gpio_type type, gpio_speed speed, gpio_resistor resistor)
{
	GPIO_TypeDef *gpio = gpio_parse_gpio(pin);
	gpio_pin_num num = gpio_parse_pin_num(pin);

	gpio_enable_clock(gpio);
	gpio_mode_select(gpio,num,OUTPUT);
    //Clear the bit
	gpio->OTYPER &= ~(0x01<<num);
	//Set output type (push pull or open drain)
    gpio->OTYPER |= (type<<num);
	//Clear the bits
	gpio->OSPEEDR &= ~(0x03<<num*0x02);
	//Set speed
	gpio->OSPEEDR &= ~(speed<<num*0x02);
	//Clear the bits
	gpio->PUPDR &= ~(0x03<<num*0x02);
	//Set the resistors
	gpio->PUPDR |= (resistor<<num*0x02);
}

/**
 * @brief Set the selected pin in input mode.
 * 
 * @param pin This parameter can be any of the values PXY (where X = A..C or H and Y = 0~15)
 * 				PXY is the friendly name of a pin as shown on the datasheet. 
 * @param resistor This parameter can be any of the values:
 *					NO_PUPD = 0x00, ///< No pull-up, pull-down
 *					PU = 0x01, ///< Pull-up
 *					PD = 0x02 ///< Pull-down
 */
void gpio_input_conf(gpio_pin pin, gpio_resistor resistor)
{
	GPIO_TypeDef *gpio = gpio_parse_gpio(pin);
	gpio_pin_num num = gpio_parse_pin_num(pin);

	gpio_enable_clock(gpio);
	gpio_mode_select(gpio,num,INPUT);
	//Clear the bits
	gpio->PUPDR &= ~(0x03<<num*0x02);
	//Set the resistors
	gpio->PUPDR |= (resistor<<num*0x02);
}
/**TO-DO*/
void gpio_alt_func_init(gpio_pin pin, gpio_type type, gpio_speed speed, gpio_resistor resistor, gpio_alt_func alt_func)
{
	GPIO_TypeDef *gpio = gpio_parse_gpio(pin);
	gpio_pin_num num = gpio_parse_pin_num(pin);

	gpio_enable_clock(gpio);
	gpio_mode_select(gpio,num,ALT_FUNCTION);
	//Set output type (push pull or open drain)
	gpio->OTYPER &= ~(0x01<<num);
    gpio->OTYPER |= (type<<num);
	//Set speed
	gpio->OSPEEDR &= ~(0x03<<num*0x02);
	gpio->OSPEEDR |= speed<<num*0x02;
	//Set the resistors
	gpio->PUPDR &= ~(0x03<<num*0x02);
	gpio->PUPDR |= (resistor<<num*0x02);
	//Set alternative functions
	if ((num>=0x00) && (num<=0x07))
	{
		gpio-> AFR[0] &= ~(0x0F<<(num*0x04));
		gpio-> AFR[0] |= alt_func<<(num*0x04);
	}
	else if ((num>=0x08) && (num<=0x0F))
	{
		gpio-> AFR[1] &= ~(0x0F<<((num-0x08)*0x04));
		gpio-> AFR[1] |= alt_func<<((num-0x08)*0x04);
	}
}

/**
 * @brief Read a selected pin current state.
 * 
 * @note This function works only if the pin is in input mode.
 * 
 * @param pin This parameter can be any of the values PXY (where X = A..C or H and Y = 0~15)
 * 				PXY is the friendly name of a pin as shown on the datasheet. 
 * @return gpio_pin_state Can be one of the :
 *			STATE_LOW = 0x00,
 *			STATE_HIGH = 0x01
 */
gpio_pin_state gpio_read(gpio_pin pin)
{
	GPIO_TypeDef *gpio = gpio_parse_gpio(pin);
	gpio_pin_num num = gpio_parse_pin_num(pin);
	if (((gpio->MODER & (0x03<<(num*0x02)))!= INPUT<<(num*0x02)))
	{
		error_handler(__FILE__,__LINE__);
	}
	if (!(gpio->IDR & 0x01<<num))
	{
		return STATE_LOW;
	}
	else
	{
		return STATE_HIGH;
	}
}

/**
 * @brief Set the pin to a selected state.
 * 
 * @note This function works only if the pin is in output or alternate function mode.
 * 
 * @param pin This parameter can be any of the values PXY (where X = A..C or H and Y = 0~15)
 * 				PXY is the friendly name of a pin as shown on the datasheet.
 * @param state gpio_pin_state This parameter can be one of the:
 *			STATE_LOW = 0x00,
 *			STATE_HIGH = 0x01
 */
void gpio_write(gpio_pin pin, gpio_pin_state state)
{
	GPIO_TypeDef *gpio = gpio_parse_gpio(pin);
	gpio_pin_num num = gpio_parse_pin_num(pin);

	if (((gpio->MODER & (0x03<<(num*0x02)))!= OUTPUT<<(num*0x02)))
	{
		error_handler(__FILE__,__LINE__);
	}

	if (state == STATE_LOW)
	{
		gpio->BSRR = 0x01<<num;
	}
	else
	{
		gpio->BSRR = 0x01<<(num+GPIO_BSRR_BR0_Pos);
	}
}

/**
 * @brief Change the pins state.
 * 
 * @note This function works only if the pin is in output or alternate function mode.
 * @param pin This parameter can be any of the values PXY (where X = A..C or H and Y = 0~15)
 * 				PXY is the friendly name of a pin as shown on the datasheet.
 */
void gpio_toggle(gpio_pin pin)
{
	GPIO_TypeDef *gpio = gpio_parse_gpio(pin);
	gpio_pin_num num = gpio_parse_pin_num(pin);


	if (((gpio->MODER & (0x03<<(num*0x02)))!= OUTPUT<<(num*0x02)))
	{
		error_handler(__FILE__,__LINE__);
	}
	if (!(gpio->ODR & 0x01<<num))
	{
		gpio->BSRR = 0x01<<num;
	}
	else
	{
		gpio->BSRR = 0x01<<(num+GPIO_BSRR_BR0_Pos);
	}
}

/**
 * @brief Enable interrupt for the selected pin
 * 
 * @param pin This parameter can be any of the values PXY (where X = A..C or H and Y = 0~15)
 * 				PXY is the friendly name of a pin as shown on the datasheet.
 * @param resistor This parameter can be any of the values:
 *					NO_PUPD = 0x00, ///< No pull-up, pull-down
 *					PU = 0x01, ///< Pull-up
 *					PD = 0x02 ///< Pull-down
 * @param edge This parameter can be any of the values:
 *			 	RISING_EDGE = 0x00,
 *				FALLING_EDGE = 0x01,
 *				RISING_FALLING_EDGE = 0x02
 * @param priority This parameter sets the interrupt priority. Lower number is higher priority.
 */
void gpio_input_it_conf(gpio_pin pin, gpio_resistor resistor, gpio_edge_trigger edge, uint32_t priority)
{
	GPIO_TypeDef *gpio = gpio_parse_gpio(pin);
	gpio_pin_num num = gpio_parse_pin_num(pin);

	IRQn_Type irq;
	//Configure as a normal input first.
	gpio_input_conf(pin,resistor);
	//Then go through with the rest of the configuration.
	//Enable the clock.
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	//Configure pin interrupt.
	if (gpio==GPIOA)
	{
		//Using 0x07 Clears the three bits on the corresponding register according to rm0394.
		//According to rm0394 all GPIOA pin bits are 000.
		SYSCFG->EXTICR[num/0x04] &= ~(0x07<<((num/0x03)*(num%0x03)));
	}
	else if (gpio==GPIOB)
	{
		SYSCFG->EXTICR[num/0x04] &= ~(0x07<<((num/0x03)*(num%0x03)));
		//According to rm0394 all GPIOB pin bits are 001.
		SYSCFG->EXTICR[num/0x04] |= (0x01<<((num/0x03)*(num%0x03)));
	}
	else if (gpio==GPIOC)
	{
		SYSCFG->EXTICR[num/0x04] &= ~(0x07<<((num/0x03)*(num%0x03)));
		//According to rm0394 all GPIOB pin bits are 010.
		SYSCFG->EXTICR[num/0x04] |= (0x02<<((num/0x03)*(num%0x03)));
	}
	else if (gpio==GPIOH)
	{
		SYSCFG->EXTICR[0] &= ~(0x07<<((num/0x03)*(num%0x03)));
		//According to rm0394 all GPIOB pin bits are 111.
		SYSCFG->EXTICR[0] |= (0x07<<((num/0x03)*(num%0x03)));
	}

	//Enable detection depending on the edge.
	if (edge == RISING_EDGE)
	{
		EXTI->FTSR1 &= ~(0x01<<num);
		EXTI->RTSR1 |= 0x01<<num;
	}
	else if (edge == FALLING_EDGE)
	{
		EXTI->FTSR1 |= 0x01<<num;
		EXTI->RTSR1 &= ~(0x01<<num);
	}
	else
	{
		EXTI->FTSR1 |= 0x01<<num;
		EXTI->RTSR1 |= 0x01<<num;
	}

	//Select EXTI IRQn depending on the output of the multiplexer.
	switch(num)
	{
		case GPIO_PIN_0:
			irq = EXTI0_IRQn;
		break;
		case GPIO_PIN_1:
			irq = EXTI1_IRQn;
		break;
		case GPIO_PIN_2:
			irq = EXTI2_IRQn;
		break;
		case GPIO_PIN_3:
			irq = EXTI3_IRQn;
		break;
		case GPIO_PIN_4:
			irq = EXTI4_IRQn;
		break;
		default :
		break;
	}
  if (num>=GPIO_PIN_5 && num<=GPIO_PIN_9)
  {
	irq = EXTI9_5_IRQn;
  }
  if (num>=GPIO_PIN_10 && num<=GPIO_PIN_15)
  {
	irq = EXTI15_10_IRQn;
  }
  //Set NVIC priority and enable the interrupt
	NVIC_SetPriority(irq,priority);
	EXTI->IMR1 |= (0x01<<num);
	NVIC_EnableIRQ(irq);
}


/************************************************
 *  Interrupt handlers.
 ***********************************************/
void EXTI0_IRQHandler()
{
	if (EXTI->PR1 & (0x01<<GPIO_PIN_0))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_0);
		gpio_interrupt_callback(GPIO_PIN_0);
	}
}

void EXTI1_IRQHandler()
{

	if (EXTI->PR1 & (0x01<<GPIO_PIN_1))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_1);
		gpio_interrupt_callback(GPIO_PIN_1);
	}
}

void EXTI2_IRQHandler()
{

	if (EXTI->PR1 & (0x01<<GPIO_PIN_2))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_2);
		gpio_interrupt_callback(GPIO_PIN_2);
	}
}

void EXTI3_IRQHandler()
{

	if (EXTI->PR1 & (0x01<<GPIO_PIN_3))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_3);
		gpio_interrupt_callback(GPIO_PIN_3);
	}
}

void EXTI4_IRQHandler()
{

	if (EXTI->PR1 & (0x01<<GPIO_PIN_4))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_4);
		gpio_interrupt_callback(GPIO_PIN_4);
	}
}

void EXTI9_5_IRQHandler()
{
	if (EXTI->PR1 & (0x01<<GPIO_PIN_5))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_5);
		gpio_interrupt_callback(GPIO_PIN_5);
	}

	if (EXTI->PR1 & (0x01<<GPIO_PIN_6))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_6);
		gpio_interrupt_callback(GPIO_PIN_6);
	}

	if (EXTI->PR1 & (0x01<<GPIO_PIN_7))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_7);
		gpio_interrupt_callback(GPIO_PIN_7);
	}

	if (EXTI->PR1 & (0x01<<GPIO_PIN_8))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_8);
		gpio_interrupt_callback(GPIO_PIN_8);
	}

	if (EXTI->PR1 & (0x01<<GPIO_PIN_9))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_9);
		gpio_interrupt_callback(GPIO_PIN_9);
	}
}

void EXTI15_10_IRQHandler()
{
	if (EXTI->PR1 & (0x01<<GPIO_PIN_10))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_10);
		gpio_interrupt_callback(GPIO_PIN_10);
	}

	if (EXTI->PR1 & (0x01<<GPIO_PIN_11))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_11);
		gpio_interrupt_callback(GPIO_PIN_11);
	}

	if (EXTI->PR1 & (0x01<<GPIO_PIN_12))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_12);
		gpio_interrupt_callback(GPIO_PIN_12);
	}

	if (EXTI->PR1 & (0x01<<GPIO_PIN_13))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_13);
		gpio_interrupt_callback(GPIO_PIN_13);
	}

	if (EXTI->PR1 & (0x01<<GPIO_PIN_14))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_14);
		gpio_interrupt_callback(GPIO_PIN_14);
	}

	if (EXTI->PR1 & (0x01<<GPIO_PIN_15))
	{
		//Clear interrupt then handle it.
		EXTI->PR1 |= (0x01<<GPIO_PIN_15);
		gpio_interrupt_callback(GPIO_PIN_15);
	}
}

/**
 * @brief Callback function can be overwritten by the user.
 * 
 */
__attribute__((weak)) void gpio_interrupt_callback(gpio_pin_num num)
{


}


/**
 * @brief Disable the selected gpio clock.
 * @note Not being used yet.
 * @param gpio 
 */
void gpio_disable_clock(GPIO_TypeDef *gpio)
{
		if (gpio == GPIOA)
		{
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOAEN;
		}
		else if (gpio == GPIOB)
		{
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOBEN;
		}
		else if (gpio == GPIOC)
		{
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOCEN;
		}
		else if (gpio ==  GPIOH)
		{
			RCC->AHB2ENR &= ~RCC_AHB2ENR_GPIOHEN;
		}
}

/**
 * @brief 
 * @note Not used yet.
 * @todo The function is not properly working yet.
 * @param num 
 * @param irq 
 */
void gpio_interrupt_disable(gpio_pin_num num, IRQn_Type irq)
{
	//Handle Pending interrupts.
	while(EXTI->PR1 & (0x01<<num))
	{

	}

	if ((num>=GPIO_PIN_0) && (num<=GPIO_PIN_4))
	{
		EXTI->IMR1 &= ~(0x01<<num);
		NVIC_DisableIRQ(irq);
	}
	else if(((num>=GPIO_PIN_5) && (num<=GPIO_PIN_9))||((num>=GPIO_PIN_10) && (num<=GPIO_PIN_15)))
	{
		//call NVIC_DisableIRQ only if there is only one line left.
		uint8_t enabled_exti_counter = 0x00;

		for (uint8_t i=0x00;i<0x05;i++)
		{
			if (EXTI->IMR1 & (0x01<<num))
			{
				enabled_exti_counter++;
			}
		}
		//disable the pin interrupt
		EXTI->IMR1 &= ~(0x01<<num);
		if (!(enabled_exti_counter>0x01))
		{
			//disable the nvic interrupt
			NVIC_DisableIRQ(irq);
		}
	}
}
