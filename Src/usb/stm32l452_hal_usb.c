#if 0

#include "stm32l452_hal_usb.h"

btable *p_btable;
static void EP_init(uint8_t EPn, uint16_t type, uint16_t kind, uint16_t rx, uint16_t tx);
static void setup_packet_handler(uint8_t EPn);

static void usb_write_USB_SRAM(uint8_t EPn, uint8_t *buffer, uint16_t size);
static void usb_read_USB_SRAM(uint8_t EPn, uint8_t *buffer, uint16_t size);

static void reconfigure_EP_OUT(uint8_t EPn, uint16_t rx);
static void reconfigure_EP_IN(uint8_t EPn, uint16_t tx);

void static handle_USB_out(uint8_t EPn);
void static handle_USB_in(uint8_t EPn, uint8_t *buffer, uint16_t size);






void usb_init()
{
    /**
     * @todo add usb clock selection of other clock sources
     * 
     */
    uint32_t start;
    //Enable HSI48 clock
    RCC->CRRCR &= ~(RCC_CRRCR_HSI48ON);
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    start = sys_get_systick();
    //Wait for hsi48 to start. If it fails to start within 2 2ms go to error.
    while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY))
    {
        if (sys_get_systick()-start>2)
        {
            error_handler(__FILE__,__LINE__);
        }
    }
    //Select clk48 source as HSI48
    RCC->CCIPR |= ~(RCC_CCIPR_CLK48SEL);
    //Enable gpio PA12 & PA11 as AF
    gpio_alt_func_init(PA11, PP, SPEED_VERY_HIGH, NO_PUPD, AF10);
    gpio_alt_func_init(PA12, PP, SPEED_VERY_HIGH, NO_PUPD, AF10);
    //Enable USB Clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_USBFSEN;
    //Validate USB Power supply.
    PWR->CR2 |= PWR_CR2_USV;
    //Set the BTABLE at the beginning of the USB_SRAM.
    USB->BTABLE = 0;

    //Create a pointer at the beginning of SRAM with the USB->BTABLE value as an offset.
    p_btable = (btable*)(USB_PMAADDR+USB->BTABLE);    
    /**
     * @brief Initialize control endpoint EP0R packet buffers inside USB_SRAM.
     * 
     */
    p_btable->USB_ADDRn_TX = 0x40; // ADDR0_TX is set to 0x4000C40
    p_btable->USB_COUNTn_TX = 0x00; // Set to 0 bytes.
    p_btable->USB_ADDRn_RX = 0x80; // ADDR0_RX is set to 0x4000C80  
    p_btable->USB_COUNTn_RX = 0x08; // Set to 8 bytes.

#if defined (USB_CLASS_CDC)
//CDC uses 2 (IN/OUT) control 1 (IN) interrupt and 1 (IN/OUT) bulk 
 /**
  * @brief Initialize interrupt endpoint (IN) EP1R packet buffers inside USB_SRAM.
  * 
  */
    (p_btable+1)->USB_ADDRn_TX = 0xC0; // ADDR1_TX is set to 0x4000CC0
    (p_btable+1)->USB_COUNTn_TX = 0x08; // Set to 8 bytes.
 /**
  * @brief Initialize bulk endpoint (IN) EP2R packet buffers inside USB_SRAM.
  * 
  */
    (p_btable+2)->USB_ADDRn_TX = 0x100; // ADDR2_TX is set to 0x4000D00
    (p_btable+2)->USB_COUNTn_TX = 0x40; // Set to 64 bytes.
/**
  * @brief Initialize bulk endpoint (OUT) EP3R packet buffers inside USB_SRAM.
  * 
  */
    (p_btable+3)->USB_ADDRn_RX = 0x140; // ADDR3_RX is set to 0x4000D40  
    (p_btable+3)->USB_COUNTn_RX = 0x40; // Set to 64 bytes.

#endif

    //Add USB Priority in the parameters of the function.
    NVIC_SetPriority(USB_IRQn, 0);
    NVIC_EnableIRQ(USB_IRQn);

    //Reset.
    USB->CNTR |= USB_CNTR_FRES;
    //Exit power down
    USB->CNTR &= ~(USB_CNTR_PDWN);
    //Wait 1us (we can only use 1 ms in our SysTick implementation) //figure out a better way later.
    sys_ms_delay(1);
    //Clear FRES
    USB->CNTR &= ~(USB_CNTR_FRES);
    //Clear the pending interrupts register.
    USB->ISTR = 0;
    //Enable the interrupts.
    USB->CNTR |= (USB_CNTR_CTRM | USB_CNTR_RESETM);
    //Enable the usb pullup
    USB->BCDR |= USB_BCDR_DPPU;

#if 0
/**
 * @brief Unsure about the following
 * 
 */
   
    //Enable power clock.
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;//?
#ifdef SELF_POWERED
    //Enable USB power monitoring
    PWR->CR2|= PWR_CR2_PVME1;
    
    start = sys_get_systick();
    //Wait for PWR_SR2_PVMO1 to clear. If it fails to start within 100ms go to error.
    while (PWR->SR2 & PWR_SR2_PVMO1)
    {
        if (sys_get_systick()-start>100)
        {
            error_handler(__FILE__,__LINE__);
        }
    }
    //Disable USB power monitoring
    PWR->CR2 &= !(PWR_CR2_PVME1);
#endif
#endif


    #if 0 
    //Enable LPM
    USB->LPMCSR |= USB_LPMCSR_LMPEN;
    USB->LPMCSR |= USB_LPMCSR_LPMACK;
    #endif


}


void USB_IRQHandler() 
{
    //Reset interrupt
    if (USB->ISTR & USB_ISTR_RESET)
    {
        //Clear the flag
        USB->ISTR &= ~(USB_ISTR_RESET);
        //Initialize Endpoint 0 as control
        EP_init(0,USB_EP_CONTROL,USB_EP_KIND,USB_EP_RX_VALID,USB_EP_TX_NAK);
        //Set address
        USB->DADDR |= USB_DADDR_EF;
    }
    if (USB->ISTR & USB_ISTR_CTR)
    {   
        //Which direction?
        uint8_t dir = USB->ISTR & USB_ISTR_DIR;
        //Which endpoint caused the interrupt?
        uint8_t EPn = USB->ISTR & USB_ISTR_EP_ID;
        //Read the register.
        volatile uint16_t *p_EPn = (uint16_t *) (USB_BASE+4*EPn);
        if (!dir)
        {
            //Is another transaction pending?

            //Do transmit stuff.
        }
        else
        {
            //Is another transaction pending?
            //Do stuff
            //Handle endpoint 0 setup packet
            if (!EPn && *p_EPn & USB_EP_SETUP)
            {
                setup_packet_handler(EPn);
            }
            //Handle data transactions.
            else
            {
                handle_USB_out(EPn);
            }
            //Do we need to do IN transaction too?
            if (*p_EPn & USB_EP_CTR_TX)
            {
                
            }

        }
    }
    #if 0
    if (USB->ISTR & USB_ISTR_PMAOVR)
    {
        //Clear the flag
        USB->ISTR &= ~(USB_ISTR_PMAOVR);  
    }
    #if DEBUG
    if (USB->ISTR & USB_ISTR_ERR)
    {   
        //Clear the flag
        USB->ISTR &= ~(USB_ISTR_ERR);   
    }
    #endif
    if (USB->ISTR & USB_ISTR_WKUP)
    {   
        //Clear the flag
        USB->ISTR &= ~(USB_ISTR_WKUP); 
    }
    if (USB->ISTR & USB_ISTR_SUSP)
    {   
        //Clear the flag
        USB->ISTR &= ~(USB_ISTR_SUSP); 
    }
    if (USB->ISTR & USB_ISTR_SOF)
    {   
        //Clear the flag
        USB->ISTR &= ~(USB_ISTR_SOF); 
    }
    if (USB->ISTR & USB_ISTR_ESOF)
    {   
        //Clear the flag
        USB->ISTR &= ~(USB_ISTR_ESOF); 
    }
    if (USB->ISTR & USB_ISTR_L1REQ)
    {
        //Clear the flag
        USB->ISTR &= ~(USB_ISTR_L1REQ); 
    }
    #endif

}

void static handle_USB_out(uint8_t EPn)
{
    uint16_t size = (p_btable+EPn)->USB_COUNTn_RX & 0x3FF;
    uint8_t *buffer = malloc(size);
    usb_read_USB_SRAM(EPn,buffer,size);
    /*********************************
     * Add user USB Callback here.
     *********************************/
    free(buffer);
    volatile uint16_t *p_En = (uint16_t *) (USB_BASE+4*EPn);
    *p_En^= USB_EP_RX_VALID;
    //Clear the flag
    *p_En &= ~(USB_EP_CTR_RX);//Fix this this is wrong  

}

void static handle_USB_in(uint8_t EPn, uint8_t *buffer, uint16_t size)
{
    usb_write_USB_SRAM(EPn,buffer,size);
    //Add user USB Callback here.
    reconfigure_EP_IN(EPn,USB_EP_TX_VALID);
    //Clear the flag
    EPn &= ~(USB_EP_CTR_TX);
}

void usb_transmit(uint8_t *buffer, uint16_t size)
{
    if (!buffer)
    {
        error_handler(__FILE__,__LINE__);
    }
    //Figure out the EP number later.
    for (uint16_t i=0;i<size;i+=64)
    {
        handle_USB_in(1,buffer,i);
    }
    if (size%64!=0)
    {
        handle_USB_in(1,buffer,size%64);
    }    
}


/**
 * @brief 
 * Setup packet should look like this.
 * 
 *  uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;

 * to make life easy use a uint8_t buffer
 * 
 * 
 * 
 */

static void setup_packet_handler(uint8_t EPn)
{
    uint8_t setup_packet_buf[8];
    usb_read_USB_SRAM(EPn,setup_packet_buf,sizeof(setup_packet_buf));
    uint8_t bmRequestType = *setup_packet_buf;
    uint8_t bRequest = *(setup_packet_buf+1);
    uint16_t wValue = ((*(setup_packet_buf+2)<<8) | *(setup_packet_buf+3));
    uint16_t wIndex = ((*(setup_packet_buf+4)<<8) | *(setup_packet_buf+5));
    uint16_t wLength = ((*(setup_packet_buf+6)<<8) | *(setup_packet_buf+7));
    uint8_t *buffer;
    uint16_t size=wLength;

 switch(bmRequestType)
    {
        //Standard Device requests
        #if 0
        case 0x00:
            switch (bRequest)
            {
                case CLEAR_FEATURE:
                break;                
                case SET_FEATURE:
                break;
                
                case SET_ADDRESS:
                break;
                case SET_DESCRIPTOR:
                break;
                case SET_CONFIGURATION:
                break;
            }
        break;
        #endif
        case 0x80:
            switch (bRequest)
            {
                case GET_STATUS:
                break;
                case GET_DESCRIPTOR:
                    switch (wValue)
                    {
                        case DESCRIPTOR_TYPE_DEVICE:

                            size = sizeof(device_descriptor);                                                       
                            p_btable->USB_ADDRn_TX = size;
                            buffer = malloc(size);
                            memcpy(buffer,device_descriptor,size);                            
                        break;
                        case DESCRIPTOR_TYPE_CONFIGURATION:
                            size = sizeof(configuration_descriptor);                                                       
                            p_btable->USB_ADDRn_TX = size;
                            buffer = malloc(size);
                            memcpy(buffer,configuration_descriptor,size);    
                        break;
                        case DESCRIPTOR_TYPE_STRING:
                        break;
                        #if 0
                        case DESCRIPTOR_TYPE_INTERFACE:
                        break;
                        case DESCRIPTOR_TYPE_ENDPOINT:
                        break;
                        case DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
                        break;
                        case DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION:
                        break;
                        case DESCRIPTOR_TYPE_INTERFACE_POWER1:
                        break;
                        #endif
                    }
                break;
                case GET_CONFIGURATION:
                break;
            }
        break;
        //Standard Interface Requests
        #if 0
        case 0x01:
            switch (bRequest)
            {
                case CLEAR_FEATURE:
                break;                
                case SET_FEATURE:
                break;
                
                case SET_INTERFACE:
                break;
            }        
        break;
        #endif
        case 0x81:
            switch (bRequest)
            {
                case GET_STATUS:
                break;
                case GET_INTERFACE:
                break;
            }        
        break;
        //Standard Endpoint Requests
        #if 0
        case 0x02:
            switch (bRequest)
            {
                case CLEAR_FEATURE:
                break;
                
                case SET_FEATURE:
                break;
                
            }            
        break;
        #endif
        case 0x82:
            switch (bRequest)
            {
                case GET_STATUS:
                break;
                case SYNCH_FRAME:
                break;
            }            
        break;
        default:
        //Reply with STALL
        break;
    }
    if (buffer)
    {
        usb_write_USB_SRAM(EPn,buffer,size);
        //EP_init(EPn,USB_EP_CONTROL,USB_EP_KIND,USB_EP_RX_NAK,USB_EP_TX_VALID);
        free(buffer);
    }





















}

static void EP_init(uint8_t EPn, uint16_t type, uint16_t kind, uint16_t rx, uint16_t tx)
{      
    //Pointer to the register
    volatile uint16_t *p_USB = (uint16_t *) (USB_BASE+4*EPn);
    //Set the register bits. (EP_TYPE, EP_KIND and EA)
    *p_USB |= (type | kind | EPn);
    *p_USB^= rx;
    *p_USB^= tx;
}



static void reconfigure_EP_IN(uint8_t EPn, uint16_t tx)
{    
    volatile uint16_t *p_USB = (uint16_t *) (USB_BASE+4*EPn);
    *p_USB^= tx;
}
static void reconfigure_EP_OUT(uint8_t EPn, uint16_t rx)
{    
    volatile uint16_t *p_USB = (uint16_t *) (USB_BASE+4*EPn);
    *p_USB^= rx;
}

/**
 * @brief Copy from the user buffer to the USB_SRAM.
 * 
 * @note User should make sure to prepare the endpoint Btable before
 *       writing to memory. No checks are done here. User buffer
 *       should be 16bit aligned.
 * @param EPn The value of the EPn parameter should be from 0-7.
 * @param data Pointer to buffer that holds the data to write to USB_SRAM.
 * @param size The value of this parameter should be 0-64 bytes.
 */
static void usb_write_USB_SRAM(uint8_t EPn, uint8_t *data, uint16_t size)
{
    //Use the USB_ADDRn_TX as the starting address
    volatile uint8_t *usb_pma_tx_buf = (uint8_t*)(USB_PMAADDR + (p_btable+EPn)->USB_ADDRn_TX);
    uint8_t len = 64;
    //Use the smallest amount of size possible.
    if (size<(p_btable+EPn)->USB_COUNTn_TX)
    {
        len = size;
    }

    for (uint8_t i=0;i<len;i++)
    {
        *(usb_pma_tx_buf+i) = *(data+i);
    }
}

/**
 * @brief Copy from the USB_SRAM to the user buffer.
 * 
 * @note User should make sure to prepare the endpoint Btable before
 *       writing to memory. No checks are done here. User buffer
 *       should be 16bit aligned.
 * @param EPn The value of the EPn parameter should be from 0-7.
 * @param data Pointer to buffer that holds the data to write to USB_SRAM.
 * @param size The value of this parameter should be 0-64 bytes.
 */
static void usb_read_USB_SRAM(uint8_t EPn, uint8_t *data, uint16_t size)
{
    //Use the USB_ADDRn_TX as the starting address
    volatile uint8_t *usb_pma_rx_buf = (uint8_t*)(USB_PMAADDR + (p_btable+EPn)->USB_ADDRn_RX);
    
    //Make sure that there is enough space in the user's buffer.    
    if (!(size<(p_btable+EPn)->USB_COUNTn_RX))
    {
        for (uint8_t i=0;i<size;i++)
        {
            *(data+i) = *(usb_pma_rx_buf+i);
        }
    }
}
#endif