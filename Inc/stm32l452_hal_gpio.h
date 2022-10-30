#ifndef STM32L452_HAL_GPIO_H
#define STM32L452_HAL_GPIO_H

#include "stm32l452_hal_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************************************
 *  Enums
 ***********************************************/

/**
 * @brief Pinout of the gpios as seen in stm32l452re datasheet.
 * 
 */
typedef enum
{
	PA0 = 0x00,
	PA1 = 0x01,
	PA2 = 0x02,
	PA3 = 0x03,
	PA4 = 0x04,
	PA5 = 0x05,
	PA6 = 0x06,
	PA7 = 0x07,
	PA8 = 0x08,
	PA9 = 0x09,
	PA10 = 0x0A,
	PA11 = 0x0B,
	PA12 = 0x0C,
	PA13 = 0x0D,
	PA14 = 0x0E,
	PA15 = 0x0F,
	PB0 = 0x10,
	PB1 = 0x11,
	PB2 = 0x12,
	PB3 = 0x13,
	PB4 = 0x14,
	PB5 = 0x15,
	PB6 = 0x16,
	PB7 = 0x17,
	PB8 = 0x18,
	PB9 = 0x19,
	PB10 = 0x1A,
	PB11 = 0x1B,
	PB12 = 0x1C,
	PB13 = 0x1D,
	PB14 = 0x1E,
	PB15 = 0x1F,
	PC0 = 0x20,
	PC1 = 0x21,
	PC2 = 0x22,
	PC3 = 0x23,
	PC4 = 0x24,
	PC5 = 0x25,
	PC6 = 0x26,
	PC7 = 0x27,
	PC8 = 0x28,
	PC9 = 0x29,
	PC10 = 0x2A,
	PC11 = 0x2B,
	PC12 = 0x2C,
	PC13 = 0x2D,
	PC14 = 0x2E,
	PC15 = 0x2F,
	PH0 = 0x30,
	PH1 = 0x31,
	PH3 = 0x33

} gpio_pin;

/**
 * @brief Pin numbers.
 * 
 */
typedef enum
{
	GPIO_PIN_0 = 0x00,
	GPIO_PIN_1 = 0x01,
	GPIO_PIN_2 = 0x02,
	GPIO_PIN_3 = 0x03,
	GPIO_PIN_4 = 0x04,
	GPIO_PIN_5 = 0x05,
	GPIO_PIN_6 = 0x06,
	GPIO_PIN_7 = 0x07,
	GPIO_PIN_8 = 0x08,
	GPIO_PIN_9 = 0x09,
	GPIO_PIN_10 = 0x0A,
	GPIO_PIN_11 = 0x0B,
	GPIO_PIN_12 = 0x0C,
	GPIO_PIN_13 = 0x0D,
	GPIO_PIN_14 = 0x0E,
	GPIO_PIN_15 = 0x0F
} gpio_pin_num;


/**
 * @brief Select the mode
 * 
 */
typedef enum
{
	INPUT = 0x00, ///<Input mode
	OUTPUT = 0x01, ///< General purpose output mode
	ALT_FUNCTION = 0x02, ///< Alternate function mode
	ANALOG = 0x03 ///< Analog mode (reset state)

} gpio_mode;

/**
 * @brief Select the configuration of the mosfets (push-pull or open drain)
 *  Can be used with output and alternative function modes only.
 * 
 */
typedef enum
{
	PP = 0x00, ///< Output push-pull (reset state)
	OD = 0x01 ///< Output open-drain

} gpio_type;

/**
 * @brief Select the speed of the gpio. Can be used with output and alternate function modes only.
 * 
 */
typedef enum
{
	SPEED_LOW = 0x00,
	SPEED_MEDIUM = 0x01,
	SPEED_HIGH = 0x02,
	SPEED_VERY_HIGH = 0x03
} gpio_speed;

/**
 * @brief Select a resistor for the gpio. Cannot be used with analog mode.
 * 
 */
typedef enum
{
	NO_PUPD = 0x00, ///< No pull-up, pull-down
	PU = 0x01, ///< Pull-up
	PD = 0x02 ///< Pull-down
} gpio_resistor;


/**
 * @brief Select at which edge the interrupt will trigger.
 * 
 */
typedef enum
{
	RISING_EDGE = 0x00,
	FALLING_EDGE = 0x01,
	RISING_FALLING_EDGE = 0x02
} gpio_edge_trigger;


/**
 * @brief Alternative function use is described in the datasheet.s
 * 
 */
typedef enum
{
	AF0 = 0x00,
	AF1 = 0x01,
	AF2 = 0x02,
	AF3 = 0x03,
	AF4 = 0x04,
	AF5 = 0x05,
	AF6 = 0x06,
	AF7 = 0x07,
	AF8 = 0x08,
	AF9 = 0x09,
	AF10 = 0x0A,
	AF11 = 0x0B,
	AF12 = 0x0C,
	AF13 = 0x0D,
	AF14 = 0x0E,
	AF15 = 0x0F
} gpio_alt_func;

/**
 * @brief Pin state.
 * 
 */
typedef enum
{
  STATE_LOW = 0x00,
  STATE_HIGH = 0x01
} gpio_pin_state;


/************************************************
 *  Functions
 ***********************************************/
void gpio_analog_conf(gpio_pin pin); ///< Reset state.
void gpio_output_conf(gpio_pin pin, gpio_type type, gpio_speed speed, gpio_resistor resistor);
void gpio_input_conf(gpio_pin pin, gpio_resistor resistor);
void gpio_alt_func_init(gpio_pin pin, gpio_type type, gpio_speed speed, gpio_resistor resistor, gpio_alt_func alt_func);
gpio_pin_state gpio_read(gpio_pin pin);
void gpio_write(gpio_pin pin, gpio_pin_state state);
void gpio_toggle(gpio_pin pin);

void gpio_input_it_conf(gpio_pin pin, gpio_resistor resistor, gpio_edge_trigger edge, uint32_t priority);
void gpio_interrupt_callback(gpio_pin_num num);


//TO-DO
void gpio_disable_clock(GPIO_TypeDef *gpio);
void gpio_interrupt_disable(gpio_pin_num num, IRQn_Type irq);


#ifdef __cplusplus
}
#endif
#endif //endif STM32L452_HAL_GPIO_H