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
 *  void clock_pll_conf(pll_clk_src clk_src,uint32_t sysclk_freq,
 *  uint32_t pll48m1clk_freq,uint32_t pllp_freq);
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
        error_handler(__FILE__,__LINE__);
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
                error_handler(__FILE__,__LINE__);
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
                error_handler(__FILE__,__LINE__);
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
            error_handler(__FILE__,__LINE__);
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
                    error_handler(__FILE__,__LINE__);
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
                error_handler(__FILE__,__LINE__);
            }
        }
    }
}

/**
 * @brief Enable or change pll configuration.
 * 
 * @note If the combination of frequencies selected by the user cannot produce a valid result there won't be any changes in pll.
 * @todo This function is too convoluted should be simplified.
 * @param clk_src This parameter sets the clock source and can be any of these values: 
 *               PLL_SRC_NONE = 0x00,
 *               PLL_SRC_MSI = 0x01,
 *               PLL_SRC_HSI = 0x02,
 *               PLL_SRC_HSE = 0x03
 * @param sysclk_freq This parameter sets the SYSCLK frequency that the user wants to achieve in Hz (8000000Hz - 80000000Hz).
 *                    If the user doesn't want to use this output the value should be set to 0.
 * @param pll48m1clk_freq This parameter sets the PLL48M1CLK frequency that the user wants to achieve in Hz (8000000Hz - 80000000Hz).
 *                        If the user doesn't want to use this output the value should be set to 0.
 * @param pllp_freq This parameter sets the PLLP frequency that the user wants to achieve in Hz (2064000Hz - 80000000Hz).
 *                  If the user doesn't want to use this output the value should be set to 0.
 * @param pllsair_freq This parameter sets the PLLSAI1R frequency that the user wants to achieve in Hz (8000000Hz - 80000000Hz).
 *                    If the user doesn't want to use this output the value should be set to 0.
 * @param pllsaiq_freq This parameter sets the PLLSAI1Q frequency that the user wants to achieve in Hz (8000000Hz - 80000000Hz).
 *                    If the user doesn't want to use this output the value should be set to 0.
 * @param pllsaip_freq This parameter sets the PLLSAI1P frequency that the user wants to achieve in Hz (2064000Hz - 80000000Hz).
 *                    If the user doesn't want to use this output the value should be set to 0.
 */
void clock_pll_conf(pll_clk_src clk_src,uint32_t sysclk_freq,uint32_t pll48m1clk_freq,uint32_t pllp_freq,uint32_t pllsair_freq,uint32_t pllsaiq_freq,uint32_t pllsaip_freq)
{
    /**
     * @note Variable src_freq is the frequency of the selected source of the pll either msi or hsi or hse,
     *       Variable pllm_reg_val is the final calculated value of pll /m that should be used in the register, (add +1 to do calulations)
     *       Variable plln_reg_val is the final calculated value of pll *n that should be used in the register,
     *       Variable pllr_reg_val is the final calculated value of pll /r that should be used in the register, (add +1 and multiply by 2 to do calulations)
     *       Variable pllq_reg_val is the final calculated value of pll /q that should be used in the register, (add +1 and multiply by 2 to do calulations)
     *       Variable pllp_reg_val is the final calculated value of pll /p that should be used in the register,
     *       Variable saiplln_reg_val is the final calculated value of pllsai1 *n that should be used in the register,
     *       Variable saipllr_reg_val is the final calculated value of pllsai1 /r that should be used in the register, (add +1 and multiply by 2 to do calulations)
     *       Variable saipllq_reg_val is the final calculated value of pllsai1 /q that should be used in the register, (add +1 and multiply by 2 to do calulations)
     *       Variable saipllp_reg_val is the final calculated value of pllsai1 /p that should be used in the register.
     */
    uint32_t src_freq = 0, pllm_reg_val = 0xFF, plln_reg_val = 0, pllr_reg_val = 0, pllq_reg_val = 0, pllp_reg_val = 0, saiplln_reg_val = 0, saipllr_reg_val = 0, saipllq_reg_val = 0, saipllp_reg_val = 0, start;

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
                error_handler(__FILE__,__LINE__);
            }
        }
        //Turn off the pll source
        RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);
    }

/**
 * @brief This huge loop iterrates through all the known values of pll and pllsai1 m, n, r, q, p values and selects them according to each of the 6 output
 *        frequencies selected by the user.
 *        The outer for loop iterrates though all m values since they are common for both pll and pllsai1, the output of m can be between 4MHz and 16MHz.
 *        The 2 nested for loops in the next indentation iterrate through all the values of n for the pll and pllsai. the output can be be between 64MHz
 *        and 344MHz for Vcore 1, and a check for each of the outputs is performed. If an output is not needed it's skipped. Following that the values 
 *        for r, q, p are iterrated one by one. 
 *        If the calculated r value matches the user's input frequency then that value is selected temporarily and a check is performed, if the outputs of
 *        the pll q and p required for the pll, if they aren't then the n value calclulated is used and the n loop exits. Then another check is performed,
 *        if the ouputs for q, p for pll and r, q and p for pllsai1 are required, if they aren't then the m value calculated is used and the m loop exits. 
 *        A similar process is repeated for each of the q, p for the pll and r,q,p for the pllsai1, however each time another check is added to make sure that
 *        the previous calculated values for each of the ouputs remains valid.
 *        If the targeted frequencies the user added don't produce a valid result the value of pllm_reg_val will remain at 0xFF.
 * 
 */
    //Calculate the m, n, r, q, p values based on user selected frequencies.
    for (uint32_t pllm = 1;pllm<=8;pllm++)
    {
        if (src_freq/pllm>=4000000 && src_freq/pllm<=16000000)
        {
            //Calculate the n, r, q, p values for pll.
            for (uint32_t plln=8;plln<=86;plln++)
            {
                uint32_t temp_vco = (src_freq/pllm)*plln;
                if (temp_vco>=64000000 && temp_vco<=344000000)
                {
                    if (sysclk_freq)
                    {
                        for (uint32_t pllr = 2;pllr<=8;pllr+=2)
                        {
                            uint32_t temp_pllr = temp_vco/pllr;
                            if (temp_pllr>=8000000 && temp_pllr<=80000000 && temp_pllr == sysclk_freq)
                            {
                                pllr_reg_val = (pllr/2)-1;
                                //Depending on which outputs the user wants to use, make sure that the calculated values for those outputs are valid and exit the loop.
                                if (!pllp_freq && !pll48m1clk_freq)
                                {
                                    plln_reg_val = plln;
                                    plln = 87;
                                }
                                if (!pllsaiq_freq && !pllsaip_freq && !pll48m1clk_freq && !pllp_freq && !pllsair_freq)
                                {
                                    pllm_reg_val = pllm-1;                                    
                                    pllm = 9;                                    
                                }
                                break;
                            }
                        }
                    }
                    if (pll48m1clk_freq)
                    {
                        for (uint32_t pllq=2;pllq<=8;pllq+=2)
                        {
                            uint32_t temp_pllq = temp_vco/pllq;
                            if (temp_pllq>=8000000 && temp_pllq<=80000000 && temp_pllq == pll48m1clk_freq)
                            {
                                pllq_reg_val = (pllq/2)-1;
                                //Depending on which outputs the user wants to use, make sure that the calculated values for those outputs are valid and exit the loop.
                                if (!pllp_freq && (!sysclk_freq || temp_vco/((pllr_reg_val+1)*2) == sysclk_freq))
                                {
                                    plln_reg_val = plln;
                                    plln =87;
                                }
                                if (!pllsair_freq && !pllsaiq_freq && !pllsaip_freq && !pllp_freq && (!sysclk_freq || temp_vco/((pllr_reg_val+1)*2) == sysclk_freq))
                                {
                                    pllm_reg_val = pllm-1;
                                    pllm = 9;
                                }
                                break;
                            }
                        }
                    }
                    if (pllp_freq)
                    {
                        for (uint32_t pllp=2;pllp<=31;pllp++)
                        {
                            uint32_t temp_pllp = temp_vco/pllp;
                            if (temp_pllp>=2064000 && temp_pllp<=80000000 && temp_pllp==pllp_freq)
                            {
                                pllp_reg_val = pllp;
                                //Depending on which outputs the user wants to use, make sure that the calculated values for those outputs are valid and exit the loop.
                                if ((!pll48m1clk_freq || temp_vco/((pllq_reg_val+1)*2)==pll48m1clk_freq) && (!sysclk_freq || temp_vco/((pllr_reg_val+1)*2) == sysclk_freq))
                                {
                                    plln_reg_val = plln;
                                    plln =87;
                                }
                                if (!pllsair_freq && !pllsaiq_freq && !pllsaip_freq && (!pll48m1clk_freq || temp_vco/((pllq_reg_val+1)*2)==pll48m1clk_freq) && (!sysclk_freq || temp_vco/((pllr_reg_val+1)*2) == sysclk_freq))
                                {
                                    pllm_reg_val = pllm-1;
                                    pllm = 9;
                                }
                                break;
                            }
                        }
                    }
                }
            }
            //Calculate the n, r, q, p values for pllsai1.
            for (uint32_t saiplln=8;saiplln<=86;saiplln++)
            {
                uint32_t temp_sai_vco = (src_freq/pllm)*saiplln;
                uint32_t temp_sysclk_freq = 0;
                uint32_t temp_pll48m1clk_freq = 0;
                uint32_t temp_pllp_freq = 0;
                if(sysclk_freq)
                {
                	temp_sysclk_freq = (src_freq/pllm)*plln_reg_val/((pllr_reg_val+1)*2);
                }
                if (pll48m1clk_freq)
                {
                	temp_pll48m1clk_freq = (src_freq/pllm)*plln_reg_val/((pllq_reg_val+1)*2);
                }
                if (pllp_freq)
                {
                	temp_pllp_freq = (src_freq/pllm)*plln_reg_val/pllp_reg_val;
                }
                if (temp_sai_vco>=64000000 && temp_sai_vco<=344000000)
                {
                    if(pllsair_freq)
                    {
                        for (uint32_t saipllr = 2;saipllr<=8;saipllr+=2)
                        {
                            uint32_t temp_saipllr = temp_sai_vco/saipllr;
                            if (temp_saipllr>=8000000 && temp_saipllr<=80000000 && temp_saipllr == pllsair_freq)
                            {
                                saipllr_reg_val = (saipllr/2)-1;
                                if (!pllsaiq_freq && !pllsaip_freq)
                                {
                                    saiplln_reg_val = saiplln;
                                    saiplln =87;
                                }
                                if ((!sysclk_freq || temp_sysclk_freq==sysclk_freq) && (!pll48m1clk_freq || temp_pll48m1clk_freq == pll48m1clk_freq) && (!pllp_freq || temp_pllp_freq == pllp_freq) && !pllsaiq_freq && !pllsaip_freq)
                                {
                                    pllm_reg_val = pllm-1;                                    
                                    pllm = 9;                                    
                                }
                                break;
                            }
                        }
                    }
                    if (pllsaiq_freq)
                    {
                        for (uint32_t saipllq=2;saipllq<=8;saipllq+=2)
                        {
                            uint32_t temp_saipllq = temp_sai_vco/saipllq;
                            if (temp_saipllq>=8000000 && temp_saipllq<=80000000 && temp_saipllq == pllsaiq_freq)
                            {
                                saipllq_reg_val = (saipllq/2)-1;
                                if (!pllsaip_freq && (!pllsair_freq || temp_sai_vco/((saipllr_reg_val+1)*2) == pllsair_freq))
                                {
                                    saiplln_reg_val = saiplln;
                                    saiplln =87;
                                }
                                if ((!sysclk_freq || temp_sysclk_freq==sysclk_freq) && (!pll48m1clk_freq || temp_pll48m1clk_freq == pll48m1clk_freq) && (!pllp_freq || temp_pllp_freq == pllp_freq) && !pllsaip_freq && (!pllsair_freq || temp_sai_vco/((saipllr_reg_val+1)*2) == pllsair_freq))
                                {
                                    pllm_reg_val = pllm-1;                                    
                                    pllm = 9;                                    
                                }
                                break;
                            }
                        }
                    }
                    if (pllsaip_freq)
                    {
                        for (uint32_t saipllp=2;saipllp<=31;saipllp++)
                        {
                            uint32_t temp_sapllp = temp_sai_vco/saipllp;
                            if (temp_sapllp>=2064000 && temp_sapllp<=80000000 && temp_sapllp==pllsaip_freq)
                            {
                                pllp_reg_val = saipllp;

                                if ((!pllsaiq_freq || temp_sai_vco/((saipllq_reg_val+1)*2) == pllsaiq_freq) && (!pllsair_freq || temp_sai_vco/((saipllr_reg_val+1)*2) == pllsair_freq))
                                {
                                    saiplln_reg_val = saiplln;
                                    saiplln =87;
                                }
                                if ((!sysclk_freq || temp_sysclk_freq==sysclk_freq) && (!pll48m1clk_freq || temp_pll48m1clk_freq == pll48m1clk_freq) && (!pllp_freq || temp_pllp_freq == pllp_freq) && (!pllsaiq_freq || temp_sai_vco/((saipllq_reg_val+1)*2) == pllsaiq_freq) && (!pllsair_freq || temp_sai_vco/((saipllr_reg_val+1)*2) == pllsair_freq))
                                {
                                    pllm_reg_val = pllm-1;                                    
                                    pllm = 9;                                    
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    //The m value of the pll should be set to a known value.
    if (pllm_reg_val != 0xFF)
    {
        //Set pll registers.
        //Disable the pll
        RCC->CR &= ~(RCC_CR_PLLON);
        start = sys_get_systick();
        while(RCC->CR & RCC_CR_PLLRDY)
        {
            if (sys_get_systick()-start>2)
            {
                error_handler(__FILE__,__LINE__);
            }
        }
        //Disable pll r, q and p outputs
        RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLQEN | RCC_PLLCFGR_PLLPEN);
        //Clear pll source, m, n, r values.
        RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLR | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLP);
        //Set pll source, m, n, r, q, p values.
        RCC->PLLCFGR |= ((clk_src<<RCC_PLLCFGR_PLLSRC_Pos) | (pllm_reg_val << RCC_PLLCFGR_PLLM_Pos) | (plln_reg_val << RCC_PLLCFGR_PLLN_Pos) | (pllr_reg_val << RCC_PLLCFGR_PLLR_Pos) | (pllq_reg_val << RCC_PLLCFGR_PLLQ_Pos) | (pllp_reg_val << RCC_PLLCFGR_PLLPDIV_Pos));
        //Enable the pll
        RCC->CR |= RCC_CR_PLLON;
        start = sys_get_systick();
        while(RCC->CR & RCC_CR_PLLRDY)
        {
            if (sys_get_systick()-start>2)
            {
                error_handler(__FILE__,__LINE__);
            }
        }
        //If the selected frequency is not 0 enable the output.
        if (sysclk_freq)
        {
            //Enable pllr
            RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
        }
        if (pll48m1clk_freq)
        {   //Enable pllq
            RCC->PLLCFGR |= RCC_PLLCFGR_PLLQEN;
        }
        //Set pllp
        if (pllp_reg_val)
        {
            //Enable pllp
            RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN;  
        }    

        //Set pllsai1 Registers.
        //Disable the pllsai1
        RCC->CR &= ~(RCC_CR_PLLSAI1ON);
        start = sys_get_systick();
        while(RCC->CR & RCC_CR_PLLSAI1RDY)
        {
            if (sys_get_systick()-start>2)
            {
                error_handler(__FILE__,__LINE__);
            }
        }
        //Disable pllsai1 r, q and p outputs
        RCC->PLLSAI1CFGR &= ~(RCC_PLLSAI1CFGR_PLLSAI1REN | RCC_PLLSAI1CFGR_PLLSAI1QEN | RCC_PLLSAI1CFGR_PLLSAI1PEN);
        //Clear pllsai1 n, r, q, p values.
        RCC->PLLSAI1CFGR &= ~(RCC_PLLSAI1CFGR_PLLSAI1N | RCC_PLLSAI1CFGR_PLLSAI1R | RCC_PLLSAI1CFGR_PLLSAI1Q | RCC_PLLSAI1CFGR_PLLSAI1P);
        //Set pllsai1 n, r, q, p values.
        RCC->PLLSAI1CFGR |= ((saiplln_reg_val << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) | (saipllr_reg_val << RCC_PLLSAI1CFGR_PLLSAI1R_Pos) | (saipllq_reg_val << RCC_PLLSAI1CFGR_PLLSAI1Q_Pos) | (saipllp_reg_val << RCC_PLLSAI1CFGR_PLLSAI1PDIV_Pos));
        //Enable the pllsai1
        RCC->CR |= RCC_CR_PLLSAI1ON;
        start = sys_get_systick();
        while(RCC->CR & RCC_CR_PLLSAI1RDY)
        {
            if (sys_get_systick()-start>2)
            {
                error_handler(__FILE__,__LINE__);
            }
        }
        //If the selected frequency is not 0 enable the output.
        if (pllsair_freq)
        {
            //Enable pllsai1 r
            RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN;
        }
        if (pllsaiq_freq)
        {   //Enable pllsai1 q
            RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1QEN;
        }
        if (pllsaip_freq)
        {
            //Enable pllsai1 p
            RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1PEN;  
        }    
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
        if ((RCC->CR & RCC_CR_PLLON) && (RCC->PLLCFGR & RCC_PLLCFGR_PLLREN))
        {
            uint32_t pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM)>>RCC_PLLCFGR_PLLM_Pos);
            uint32_t plln = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN)>>RCC_PLLCFGR_PLLN_Pos);
            uint32_t pllr = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLR)>>RCC_PLLCFGR_PLLR_Pos);
            src_freq = ((clock_get_pll_src_freq()/(pllm+1))*plln)/(2*(1+pllr));
        }
    }
    //Calculate the prescaler value
    for (uint32_t i=0;i<=8;i++)
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
        uint32_t ws = target_freq/16000000;
        //Reduce the wait state to the correct one.  
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
        //Reduce the wait state to the correct one.
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
    if (!(ahb_presc_val>>0x03))
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

/**
 * @brief Enable or change the peripheral clock frequency.
 * 
 * @note Peripheral prescaler value is calculated automatically, if the target frequency is
 *       incorrect a prescaler value of 0 (peripheral frequency = HCLK) is selected.
 * 
 * @param apb_per This parameter allows the choice between APB1 or ABP2 peripheral clock 
 *                and can be any of the values:
 * 	              APB_SEL_1 = 0x01,
 *	              APB_SEL_2 = 0x02
 * @param target_freq This parameter can be equal to HCLK or HCLK devided by the prescaler.
 */
void clock_periph_conf(apb_per_sel apb_per, uint32_t target_freq)
{
    uint32_t apb_presc = 0;
    for (uint32_t i=0;i<=3;i++)
    {
        if (SystemCoreClock/(1<<(i+1)) == target_freq)
        {
            apb_presc = 0x04 | i;
            break;
        }
    }

    if(apb_per==APB_SEL_1)
    {
            //Set prescaler.
            RCC->CFGR &= ~(RCC_CFGR_PPRE1);
            RCC->CFGR |= apb_presc<<RCC_CFGR_PPRE1_Pos;
    }
    else
    {
        //Set prescaler.
        RCC->CFGR &= ~(RCC_CFGR_PPRE2);
        RCC->CFGR |= apb_presc<<RCC_CFGR_PPRE2_Pos;
    }
}

/**
 * @brief Get the frequency of a peripheral.
 * 
 * @param apb_per This parameter allows the choice between APB1 or ABP2 peripheral clock 
 *                and can be any of the values:
 * 	              APB_SEL_1 = 0x01,
 *	              APB_SEL_2 = 0x02
 * @return uint32_t Frequency in Hz.
 */
uint32_t clock_get_periph_freq(apb_per_sel apb_per)
{
    uint32_t apb_presc = 0;

    //Get the prescaler value for the selected peripheral.
    if(apb_per==APB_SEL_1)
    {
        apb_presc = ((RCC->CFGR & RCC_CFGR_PPRE1)>>RCC_CFGR_PPRE1_Pos);
    }
    else
    {
        apb_presc = ((RCC->CFGR & RCC_CFGR_PPRE2)>>RCC_CFGR_PPRE2_Pos);
    }
    
    if (!(apb_presc & 0x04))
    {
        return SystemCoreClock;
    }
    else
    {
        return SystemCoreClock/(1<<((apb_presc & 0x03)+1));
    }

}