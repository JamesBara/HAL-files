/**************************************************************************************
 *  How to use:
 *  
 *  The user should first select the oscillators that they want to enable by
 *  calling the functions:
 *  clock_enable_hsi(); ///<Enable hsi oscillator.
 *  clock_conf_msi(uint32_t msi_freq); ///<Enable msi oscillator or change frequency.
 *  clock_conf_hse(uint32_t hse_freq, hse_clk_by hse_by); ///<Enable hse oscillator. 
 *   
 *  If the user wants to enable the pll they need to call the function 
 *  void clock_pll_conf(pll_clk_src clk_src,uint32_t pllr_target_freq,
 *  uint32_t pllq_target_freq,uint32_t pllp_target_freq);
 *  @todo Implement pllsai1 in the above function. Add Vcore range 2.
 *  After activating the oscillators and/or the pll, in order to set the system clock
 *  (HCLK) the user should call the function:
 *  void clock_hclk_conf(sys_clk_src clk_src, hse_css css_enable, uint32_t target_freq); 
 *  @todo Add Vcore range 2.
 *  @note All of the functions are taking in account only Vcore range 1 frequencies, and
 *  may or may not work properly with Vcore range 2.
 * 
 ***************************************************************************************/
#include "stm32l452_hal_clock.h"

static uint32_t clock_get_msi_freq();
static uint32_t clock_get_pll_src_freq();

/**
 * @brief Get the current msi oscillator frequency.
 * 
 * @return uint32_t The msi oscillator frequency in Hz.
 */
static uint32_t clock_get_msi_freq()
{
    if (!(RCC->CR & RCC_CR_MSIRGSEL))
    {
        return MSIRangeTable[((RCC->CSR & RCC_CSR_MSISRANGE)>>RCC_CSR_MSISRANGE_Pos)];
    }
    else
    {
        return MSIRangeTable[((RCC->CR & RCC_CR_MSIRANGE)>>RCC_CR_MSIRANGE_Pos)];
    }
}

/**
 * @brief Get the current pll source oscillator frequency.
 * 
 * @return uint32_t The pll source oscillator frequency in Hz.
 */
static uint32_t clock_get_pll_src_freq()
{
    uint32_t src_freq = 0;
    if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC)==RCC_PLLCFGR_PLLSRC_HSI)
    {
        src_freq = 16000000;
    }
    else if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC)==RCC_PLLCFGR_PLLSRC_MSI)
    {
        src_freq = clock_get_msi_freq();
    }
    else if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC)==RCC_PLLCFGR_PLLSRC_HSE)
    {
        src_freq = hse_frequency;
    }
    else
    {
        //Shouldn't be here
        error_handler();
    }
        return src_freq;
}

/**
 * @brief Get the HCLK clock frequency.
 * 
 * @return uint32_t The NVIC SystemCoreClock frequency in Hz.
 */
uint32_t clock_get_hclk_freq()
{
    return SystemCoreClock;
}

/**
 * @brief Get the SYSCLK clock frequency.
 * 
 * @return uint32_t The SYSCLK clock frequency in Hz.
 */
uint32_t clock_get_sysclk_freq()
{
    //If the selected system clock is hsi return hsi value.
    if ((RCC->CFGR & RCC_CFGR_SWS)==RCC_CFGR_SWS_HSI)
    {
        return 16000000;
    }
    //If the selected system clock is msi find the msi value from the appropriate register and return it.
    else if ((RCC->CFGR & RCC_CFGR_SWS)==RCC_CFGR_SWS_MSI)
    {
        return clock_get_msi_freq();
    }
    //If the selected system clock is hse return the user defined value of hse.
    else if ((RCC->CFGR & RCC_CFGR_SWS)==RCC_CFGR_SWS_HSE)
    {
        return hse_frequency;
    }

    //If the selected system clock is pll find the source of the pll from registers, and calculate the system value.
    else if ((RCC->CFGR & RCC_CFGR_SWS)==RCC_CFGR_SWS_PLL)
    {
        uint32_t pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM)>>RCC_PLLCFGR_PLLM_Pos);
        uint32_t plln = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN)>>RCC_PLLCFGR_PLLN_Pos);
        uint32_t pllr = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLR)>>RCC_PLLCFGR_PLLR_Pos);

        return ((clock_get_pll_src_freq()/pllm)*plln)/pllr;
    }
    else
    {
        //Shouldn't be here
        return 0;
    }
}

/**
 * @brief Enable The hsi oscillator.
 * 
 */
void clock_enable_hsi()
{
    sys_set_systick();

    //If HSI is off turn it on.
    if (!(RCC->CR & RCC_CR_HSION))
    {
        //Turn on HSI.
        RCC->CR |= RCC_CR_HSION;
        uint32_t start = sys_get_systick();
        //Wait for hsi to start. If it fails to start within 2 secs go to error.
        while (!(RCC->CR & RCC_CR_HSIRDY))
        {
            if (sys_get_systick()-start>2)
            {
                error_handler();
            }
        }
             
    }
}

/**
 * @brief Enable the msi oscillator or change the frequency.
 * 
 * @note if an invalid parameter is entered the default clock of 4Mhz is selected.
 * @param msi_freq This parameter sets the msi oscillator frequency in Hz. 
 */
void clock_conf_msi(uint32_t msi_freq)
{
    uint8_t msi_freq_range = 6;
    sys_set_systick();
    uint32_t start;
    switch (msi_freq)
    {
        case 100000:
            msi_freq_range = 0;
        break;
        case 200000:
            msi_freq_range = 1;
        break;
        case 400000:
            msi_freq_range = 2;
        break;        
        case 800000:
            msi_freq_range = 3;
        break;        
        case 1000000:
            msi_freq_range = 4;
        break;        
        case 2000000:
            msi_freq_range = 5;
        break;        
        case 8000000:
            msi_freq_range = 7;
        break;        
        case 16000000:
            msi_freq_range = 8;
        break;
        case 24000000:
            msi_freq_range = 9;
        break;
        case 32000000:
            msi_freq_range = 10;
        break;
        case 48000000:
            msi_freq_range = 11;
        break;
        default:
            msi_freq_range = 6;
        break;
    }

    //If msi clock is on, turn it off and change it's range. (Test it first we may not need to turn it off,only test if it's ready.)
    if (RCC->CR & RCC_CR_MSION)
    {
        RCC->CR &= ~(RCC_CR_MSION);        
        start = sys_get_systick();
        while (!(RCC->CR & RCC_CR_MSIRDY))
        {
            if (sys_get_systick()-start>2)
            {
                error_handler();
            }
        }
    }
    //Set  msirgsel
    RCC->CR |= RCC_CR_MSIRGSEL;
    //Set clock
    RCC->CR &= ~(RCC_CR_MSIRANGE);
    RCC->CR |= msi_freq_range<<RCC_CR_MSIRANGE_Pos;
    //Turn on MSI.
    RCC->CR |= RCC_CR_MSION;
    start = sys_get_systick();
    while (!(RCC->CR & RCC_CR_MSIRDY))
    {
        if (sys_get_systick()-start>2)
        {
            error_handler();
        }
    }
    //If HCLK is set to msi, update SystemCoreClock.
    if (((RCC->CFGR & RCC_CFGR_SWS)>>RCC_CFGR_SWS_Pos)==RCC_CFGR_SWS_MSI)
    {
        SystemCoreClock = clock_get_msi_freq();
    }

}

/**
 * @brief Enable the hse oscillator.
 * 
 * @param hse_freq This parameter sets the hse oscillator frequency in Hz. 
 * @param hse_by   This parameter can be any of the values:
 *                 CLK_HSE_NONE = 0x00, ///< Use oscillator.
 *                 CLK_HSE_BYP = 0x01 ///< Use another external clock source.
 */
void clock_conf_hse(uint32_t hse_freq, hse_clk_by hse_by)
{
    sys_set_systick();
    uint32_t start;
    //set the frequency of the crystal or external input.
    if (hse_freq>=4000000 && hse_freq<=48000000)
    {
        hse_frequency = hse_freq;
        //If hse clock is on turn it off.
        if (RCC->CR & RCC_CR_HSEON)
        {
            RCC->CR &= ~(RCC_CR_HSEON);

            start = sys_get_systick();
            while (!(RCC->CR & RCC_CR_HSERDY))
            {
                if (sys_get_systick()-start>100)
                {
                    error_handler();
                }
            }
        }
        //Set bypass
        RCC->CR |= hse_by<<RCC_CR_HSEBYP_Pos;
        //Turn hse back on.
        RCC->CR |= RCC_CR_HSEON;
        while (!(RCC->CR & RCC_CR_HSERDY))
        {
            if (sys_get_systick()-start>100)
            {
                error_handler();
            }
        }
    }
}

/**
 * @brief Enable or change pll configuration.
 * 
 * @note If the combination of frequencies selected by the user cannot set valid pllr, pllq and pllp register values 
 *       the pll won't activate.
 * @param clk_src This parameter sets the clock source and can be any of these values: 
 *               PLL_SRC_NONE = 0x00,
 *               PLL_SRC_MSI = 0x01,
 *               PLL_SRC_HSI = 0x02,
 *               PLL_SRC_HSE = 0x03
 * @param pllr_target_freq This parameter sets the SYSCLK frequency that the user wants to achieve in Hz. (8000000Hz - 80000000Hz)
 * @param pllq_target_freq This parameter sets the PLL48M1CLK frequency that the user wants to achieve in Hz. (8000000Hz - 80000000Hz)
 *                         If the user doesn't want to use this output the value should be set to 0.
 * @param pllp_target_freq This parameter sets the PLLSAICLK frequency that the user wants to achieve in Hz. (2064000Hz - 80000000Hz)
 *                         If the user doesn't want to use this output the value should be set to 0.
 */
void clock_pll_conf(pll_clk_src clk_src,uint32_t pllr_target_freq,uint32_t pllq_target_freq,uint32_t pllp_target_freq)
{
    uint32_t src_freq = 0, pllm_reg_val = 0xFF, plln_reg_val = 0xFF, pllr_reg_val = 0xFF, pllq_reg_val = 0xFF, pllp_reg_val = 0xFF, vco, start;

    //Figure out the pll source and return the current frequency of the source.
    //If the clock is not enabled do nothing.
    if (clk_src == PLL_SRC_MSI)
    {
        if (RCC->CR & RCC_CR_MSION)
        {
            src_freq = clock_get_msi_freq();
        }
    }
    else if (clk_src == PLL_SRC_HSI)
    {
        if (RCC->CR & RCC_CR_HSION)
        {
            src_freq = 16000000;
        }

    }
    else if (clk_src == PLL_SRC_HSE)
    {
        if (RCC->CR & RCC_CR_HSEON)
        {
            src_freq = hse_frequency;
        }
    }
    else if (clk_src == PLL_SRC_NONE)
    {
        //Disable the pll
        RCC->CR &= ~(RCC_CR_PLLON);
        start = sys_get_systick();
        while(RCC->CR & RCC_CR_PLLRDY)
        {
            if (sys_get_systick()-start>100)
            {
                error_handler();
            }
        }
        //Turn off the pll source
        RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);
    }

    //Calculate the prescaler and vco values based on target frequency.
    for (uint32_t pllm = 1;pllm<=8;pllm++)
    {
        if (src_freq/pllm>=4000000 && src_freq/pllm<=16000000)
        {
            for (uint32_t plln=8;plln<=86;plln++)
            {
                if ((src_freq/pllm)*plln>=64000000 && (src_freq/pllm)*plln<=344000000)
                {
                    for (uint32_t pllr = 2;pllr<=8;pllr+=2)
                    {
                        if (((src_freq/pllm)*plln)/pllr>=8000000 && ((src_freq/pllm)*plln)/pllr<=80000000 && ((src_freq/pllm)*plln)/pllr == pllr_target_freq)
                        {
                            pllr_reg_val = (pllr/2)-1;
                            if (!pllq_target_freq && !pllp_target_freq)
                            {
                                pllm_reg_val = pllm-1;
                                plln_reg_val = plln;
                                pllm = 9;
                                plln =87;
                            }
                            break;
                        }
                    }

                    if (pllq_target_freq)
                    {
                        for (uint32_t pllq=2;pllq<=8;pllq+=2)
                        {
                            if (((src_freq/pllm)*plln)/pllq>=8000000 && ((src_freq/pllm)*plln)/pllq<=80000000 && ((src_freq/pllm)*plln)/pllq == pllq_target_freq)
                            {
                                pllq_reg_val = (pllq/2)-1;
                                if ((!pllp_target_freq || ((src_freq/pllm)*plln)/pllp_reg_val==pllp_target_freq) && ((src_freq/pllm)*plln)/pllr_reg_val == pllr_target_freq)
                                {
                                    pllm_reg_val = pllm-1;
                                    plln_reg_val = plln;
                                    pllm = 9;
                                    plln =87;
                                }
                                break;
                            }
                        }
                    }

                    if (pllp_target_freq)
                    {
                        for (uint32_t pllp=2;pllp<=31;pllp++)
                        {
                            if (((src_freq/pllm)*plln)/pllp>=2064000 && ((src_freq/pllm)*plln)/pllp<=80000000 && ((src_freq/pllm)*plln)/pllp==pllp_target_freq)
                            {
                                pllp_reg_val = pllp;
                                if ((!pllq_target_freq || ((src_freq/pllm)*plln)/pllq_reg_val==pllq_target_freq) && ((src_freq/pllm)*plln)/pllr_reg_val == pllr_target_freq)
                                {
                                    pllm_reg_val = pllm-1;
                                    plln_reg_val = plln;
                                    pllm = 9;
                                    plln =87;
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    //Test the result validity.
    if (pllm_reg_val!= 0xFF && plln_reg_val!=0xFF)
    {
        vco = (src_freq/(pllm_reg_val+1))*plln_reg_val;
    }

    if (pllr_reg_val!=0xFF && vco/((pllr_reg_val+1)*2)==pllr_target_freq)
    {
        //We have everything we need. Time to set the registers.
        //Disable the pll
        RCC->CR &= ~(RCC_CR_PLLON);
        
        start = sys_get_systick();
        while(RCC->CR & RCC_CR_PLLRDY)
        {
            if (sys_get_systick()-start>2)
            {
                error_handler();
            }
        }
        //Set pll source
        RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);
        RCC->PLLCFGR |= clk_src<<RCC_PLLCFGR_PLLSRC_Pos;
        //Set pllm
        RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);
        RCC->PLLCFGR |= pllm_reg_val << RCC_PLLCFGR_PLLM_Pos;
        //Set plln
        RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN);
        RCC->PLLCFGR |= plln_reg_val << RCC_PLLCFGR_PLLN_Pos;
        //Set pllr
        RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLR);
        RCC->PLLCFGR |= pllr_reg_val << RCC_PLLCFGR_PLLR_Pos;
        //Set pllq
        if (pllq_target_freq && pllq_reg_val!=0xFF && vco/((pllq_reg_val+1)*2)==pllq_target_freq)
        {
            RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLQ);
            RCC->PLLCFGR |= pllq_reg_val << RCC_PLLCFGR_PLLQ_Pos;
        }
        //Set pllp
        if (pllp_target_freq && (pllp_reg_val!=0xFF && vco/pllp_reg_val==pllp_target_freq))
        {
            RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLPDIV);
            RCC->PLLCFGR |= pllp_reg_val << RCC_PLLCFGR_PLLPDIV_Pos;
        }
        //Enable the pll
        RCC->CR |= RCC_CR_PLLON;
        start = sys_get_systick();
        while(RCC->CR & RCC_CR_PLLRDY)
        {
            if (sys_get_systick()-start>2)
            {
                error_handler();
            }
        }

    }
    //Enable pllr
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;

    //Enable pllq
    if (pllq_target_freq)
    {
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLQEN;
    }

    //Enable pllp
    if (pllp_target_freq)
    {
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN;
    }
}

/**
 * @brief Enable or change HCLK configuration.
 * 
 * @param clk_src This parameter sets the SYSCLK source and can be any of these values: 
 *               SYSCLK_SRC_NONE = 0x00,
 *               SYSCLK_SRC_MSI = 0x01,
 *               SYSCLK_SRC_HSI = 0x02,
 *               SYSCLK_SRC_HSE = 0x03
 * @param css_enable This parameter can be any of these values:
 *                   CSS_OFF = 0x00, ///< Don't use Clock Security System.
 *                   CSS_ON = 0x01 ///< Use Clock Security System.
 * @param target_freq This parameter sets the HCLK frequency that the user wants to achieve in Hz. (target_freq<=80000000Hz)
 *                  
 */
void clock_hclk_conf(sys_clk_src clk_src, hse_css css_enable, uint32_t target_freq)
{
    uint32_t current_hclk_freq = 0, src_freq = 0,ahb_presc_val = 0xFF;
    uint32_t ahb_presc[] = {1,2,4,8,16,64,128,256,512};

    //Figure out the system clock source and calculate the sysclk frequency that the clock is changing to.
    if (clk_src == SYSCLK_SRC_MSI)
    {
        if (RCC->CR & RCC_CR_MSION)
        {
            src_freq = clock_get_msi_freq();
        }
    }
    else if (clk_src == SYSCLK_SRC_HSI)
    {
        if (RCC->CR & RCC_CR_HSION)
        {
            src_freq = 16000000;
        }

    }
    else if (clk_src == SYSCLK_SRC_HSE)
    {
        if (RCC->CR & RCC_CR_HSEON)
        {
            src_freq = hse_frequency;
        }
    }
    else if (clk_src == SYSCLK_SRC_PLL)
    {
        if ((RCC->CR & RCC_CR_PLLON) && (RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN))
        {
            uint32_t pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM)>>RCC_PLLCFGR_PLLM_Pos);
            uint32_t plln = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN)>>RCC_PLLCFGR_PLLN_Pos);
            uint32_t pllr = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLR)>>RCC_PLLCFGR_PLLR_Pos);
            src_freq = ((clock_get_pll_src_freq()/pllm)*plln)/pllr;
        }
    }
    //Calculate the prescaler value
    for (uint32_t i=0;i<9;i++)
    {
        if (src_freq/ahb_presc[i]<=80000000 && src_freq/ahb_presc[i]==target_freq)
        {
            if (!i)
            {
                ahb_presc_val = i;
            }
            else
            {
                ahb_presc_val = 0x8 | (i-1);
            }
            break;
        }
    }
    //Calculate current hclk
    if (!((RCC->CFGR & RCC_CFGR_HPRE>>RCC_CFGR_HPRE_Pos)>>3))
    {
        current_hclk_freq = clock_get_sysclk_freq();
    }
    else
    {
        current_hclk_freq = clock_get_sysclk_freq()/ahb_presc[(((RCC->CFGR & RCC_CFGR_HPRE>>RCC_CFGR_HPRE_Pos)&(~0x08))+1)];
    }
    //We have everything we need. Time to set the registers.
    //Set css
    if (src_freq && ((clk_src == SYSCLK_SRC_HSE) || ((clk_src == SYSCLK_SRC_PLL) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC)==RCC_PLLCFGR_PLLSRC_HSE))))
    {
        RCC->CR |= css_enable<<RCC_CR_CSSON_Pos;
    }
    //Decreasing frequency
    if (current_hclk_freq>target_freq && src_freq)
    {
        //Set the source clock
        RCC->CFGR &= ~(RCC_CFGR_SW);
        RCC->CFGR |= clk_src<<RCC_CFGR_SW_Pos;

        //Set the prescaler value
        RCC->CFGR &= ~(RCC_CFGR_HPRE);
        RCC->CFGR |= ahb_presc_val<<RCC_CFGR_HPRE_Pos;
        //Enable prefetch
        FLASH->ACR &= ~(FLASH_ACR_PRFTEN);
        FLASH->ACR |= ~(FLASH_ACR_PRFTEN);

        //Set wait states
        uint32_t ws = target_freq/16000000;;
        if (target_freq == 16000000 || target_freq==32000000 || target_freq==48000000 || target_freq==64000000 || target_freq==80000000)
        {
            ws--;
        }
   
        FLASH->ACR &= ~(FLASH_ACR_LATENCY);
        FLASH->ACR |= (ws)<<FLASH_ACR_LATENCY_Pos;
    }
    //Increasing frequency
    else if(current_hclk_freq<target_freq && src_freq)
    {
        //Enable prefetch
        FLASH->ACR &= ~(FLASH_ACR_PRFTEN);
        FLASH->ACR |= ~(FLASH_ACR_PRFTEN);

        //Set wait states
        uint32_t ws = target_freq/16000000;
        if (target_freq == 16000000 || target_freq==32000000 || target_freq==48000000 || target_freq==64000000 || target_freq==80000000)
        {
            ws--;
        }

        FLASH->ACR &= ~(FLASH_ACR_LATENCY);
        FLASH->ACR |= ws<<FLASH_ACR_LATENCY_Pos;

        //Set the source clock
        RCC->CFGR &= ~(RCC_CFGR_SW);
        RCC->CFGR |= clk_src<<RCC_CFGR_SW_Pos;

        //Set the prescaler value
        RCC->CFGR &= ~(RCC_CFGR_HPRE);
        RCC->CFGR |= ahb_presc_val<<RCC_CFGR_HPRE_Pos;
    }
    //Make sure the clock is properly set. Then use it to set the NVIC clock.
    if (!ahb_presc_val>>0x03)
    {
        if (src_freq == target_freq)
        {
            //Set NVIC clock.
            SystemCoreClock = target_freq;
            //initialize ticks.
            sys_set_systick();
        }
    }
    else
    {
        if (src_freq/ahb_presc[((ahb_presc_val+1)&0x07)] == target_freq)
        {
            //Set NVIC clock.
            SystemCoreClock = target_freq;
            //initialize ticks.
            sys_set_systick();
        }
    }

}
