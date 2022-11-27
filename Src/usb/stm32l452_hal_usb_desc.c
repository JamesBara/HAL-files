#if 0

#include "stm32l452_hal_usb_desc.h"












#if defined(USB_CLASS_OTHER)
//EP_number = ;
#elif defined(USB_CLASS_AUDIO)

#elif defined(USB_CLASS_MSC)

#elif defined(USB_CLASS_HID)

#elif defined (USB_CLASS_CDC)
/**
 * @brief CDC descriptors
 * 
 */

uint8_t device_descriptor[] __attribute__((__aligned__(2))) =
{
    0x12, ///<bLength
    0x01, ///<bDescriptorType
    0x01, ///<bcdUSB lower byte (may change to 0) version 2.01
    0x02, ///<bcdUSB higher byte
    0x02, ///<bDeviceClass (CDC)
    0x00, ///<bDeviceSubClass
    0x00, ///<bDeviceProtocol
    0x40, ///<bMaxPacketSize
    0x83, ///<idVendor lower byte 0x483 is STMicroelectronics VID
    0x04, ///<idVendor higher byte
    0x40, ///<idProduct lower byte 0x483 is STMicroelectronics PID for CDC
    0x57, ///<idProduct higher byte
    0x00, ///<bcdDevice lower byte version 2.00
    0x02, ///<bcdDevice higher byte
    0x01, ///<iManufacturer
    0x02, ///<iProduct
    0x03, ///<iSerialNumber
    0x01 ///<bNumConfigurations
};

/**
 * @brief 
 * @note IAR is not used.
 * 
 */
uint8_t configuration_descriptor[] __attribute__((__aligned__(2))) =
{

    0x09, ///<bLength
    0x02, ///<bDescriptorType
    0x43, ///<wTotalLength lower byte
    0x00, ///<wTotalLength higher byte
    0x02, ///<bNumInterfaces
    0x01, ///<bConfigurationValue
    0x00, ///<iConfiguration //Should I add a string descriptoer?
    0x80, ///<bmAttributes D7: Reserved (set to one) D6: Self-powered D5: Remote Wakeup D4...0: Reserved (reset to zero)
    0x32, ///<bMaxPower (100mA selected (50*2mA))(Set to 0 for self-powered)

/**
 * @note Interface 1 descriptor
 * 
 */
    0x09, ///<bLength
    0x04, ///<bDescriptorType
    0x00, ///<bInterfaceNumber
    0x00, ///<bAlternateSetting
    0x01, ///<bNumEndpoints
    0x02, ///<bInterfaceClass
    0x02, ///<bInterfaceSubClass
    0x01, ///<bInterfaceProtocol
    0x00, ///<iInterface *Should I add string descriptors?

/**
 * @note Header descriptor
 * 
 */
    0x05, ///<bLength
    0x24, ///<bDescriptorType
    0x00, ///<bDescriptorSubtype
    0x10, ///<bcdCDC lower byte
    0x01, ///<bcdCDC higher byte
/**
 * @note Call management descriptor
 * 
 */
    0x05, ///<bLength
    0x24, ///<bDescriptorType
    0x01, ///<bDescriptorSubtype
    0x00, ///<bmCapabilities
    0x01, ///<bDataInterface
/**
 * @note Abstract Control Management descriptor
 * 
 */
    0x04, ///<bLength
    0x24, ///<bDescriptorType
    0x02, ///<bDescriptorSubtype
    0x02, ///<bmCapabilities Device supports the request combination of Set_Line_Coding, Set_Control_Line_State, Get_Line_Coding, and the notification Serial_State. 
/**
 * @note Union descriptor
 * 
 */
    0x05, ///<bLength
    0x24, ///<bDescriptorType
    0x06, ///<bDescriptorSubtype
    0x00, ///<bControlInterface
    0x01, ///<bSubordinateInterface0

/**
 * @note Should I add a Country Selection Functional Descriptor here?
 * 
 */
    
/**
 * @note Endpoint 1 descriptor
 * 
 */
    0x07, ///<bLength
    0x05, ///<bDescriptorType
    0x81, ///<bEndpointAddress
    0x03, ///<bmAttributes
    0x08, ///<wMaxPacketSize lower byte
    0x00, ///<wMaxPacketSize higher byte
    0x00, ///<bInterval
/**
 * @note Interface 2 descriptor
 * 
 */
    0x09, ///<bLength
    0x04, ///<bDescriptorType
    0x01, ///<bInterfaceNumber
    0x00, ///<bAlternateSetting
    0x02, ///<bNumEndpoints
    0x0A, ///<bInterfaceClass
    0x00, ///<bInterfaceSubClass
    0x00, ///<bInterfaceProtocol 
    0x00, ///<iInterface *Should I add string descriptors?
/**
 * @note Endpoint 2 descriptor
 * 
 */
    0x07, ///<bLength
    0x05, ///<bDescriptorType
    0x82, ///<bEndpointAddress
    0x02, ///<bmAttributes
    0x40, ///<wMaxPacketSize lower byte
    0x00, ///<wMaxPacketSize higher byte
    0x00, ///<bInterval
/**
 * @note Endpoint 3 descriptor
 * 
 */
    0x07, ///<bLength
    0x05, ///<bDescriptorType
    0x03, ///<bEndpointAddress
    0x02, ///<bmAttributes
    0x40, ///<wMaxPacketSize lower byte
    0x00, ///<wMaxPacketSize higher byte
    0x10 ///<bInterval
};
#endif
#endif