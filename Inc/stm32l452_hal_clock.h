#ifndef STM32L452_HAL_CLOCK_H
#define STM32L452_HAL_CLOCK_H

#include "stm32l452_hal_sys.h"
#include "stm32l452_hal_dbg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	APB_SEL_1 = 0x01,
	APB_SEL_2 = 0x02
}apb_per_sel;

typedef enum
{
	PLL_SRC_NONE = 0x00,
	PLL_SRC_MSI = 0x01,
	PLL_SRC_HSI = 0x02,
	PLL_SRC_HSE = 0x03
}pll_clk_src;

typedef enum
{
	SYSCLK_SRC_MSI = 0x00,
	SYSCLK_SRC_HSI = 0x01,
	SYSCLK_SRC_HSE = 0x02,
    SYSCLK_SRC_PLL = 0x03
} sys_clk_src;

/**
 * @brief select between the use of external oscillator or another source. 
 * 
 */
typedef enum
{
    CLK_HSE_NONE = 0x00, ///< Use oscillator.
	CLK_HSE_BYP = 0x01 ///< Use another external clock source.
} hse_clk_by;

/**
 * @brief select between the use of external oscillator or another source. 
 * 
 */
typedef enum
{
    CSS_OFF = 0x00, ///< Don't use Clock Security System.
	CSS_ON = 0x01 ///< Use Clock Security System.
} hse_css;

/************************************************
 *  Functions
 ***********************************************/
void clock_enable_hsi();
void clock_conf_msi(uint32_t msi_freq); //Select 100000Hz-48000000Hz.
void clock_conf_hse(uint32_t hse_freq, hse_clk_by hse_by); //Select 4000000Hz-48000000Hz.
uint32_t clock_get_sysclk_freq();
void clock_pll_conf(pll_clk_src clk_src,uint32_t sysclk_freq,uint32_t pll48m1clk_freq,uint32_t pllp_freq,uint32_t pllsair_freq,uint32_t pllsaiq_freq,uint32_t pllsaip_freq);
void clock_hclk_conf(sys_clk_src clk_src, hse_css css_enable, uint32_t target_freq);
void clock_periph_conf(apb_per_sel apb_per, uint32_t target_freq);
uint32_t clock_get_periph_freq(apb_per_sel apb_per);



#ifdef __cplusplus
}
#endif
#endif //endif STM32L452_HAL_CLOCK_H
