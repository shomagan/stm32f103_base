/////////////////////////////////////////////////////////////
//// Auto generate by TeenyDT, http://dt.tusb.org
/////////////////////////////////////////////////////////////
#include "teeny_usb.h"

#define  CDC_ACM_DEVICE_DESCRIPTOR_SIZE  (18)
__ALIGN_BEGIN  const uint8_t CDC_ACM_DeviceDescriptor [18] __ALIGN_END = {
  ///////////////////////////////////////
  /// device descriptor
  ///////////////////////////////////////
  0x12,                                             /* bLength */
  USB_DEVICE_DESCRIPTOR_TYPE,                       /* bDescriptorType */
  0x00, 0x02,                                       /* bcdUSB */
  0xef,                                             /* bDeviceClass */
  0x02,                                             /* bDeviceSubClass */
  0x01,                                             /* bDeviceProtocol */
  0x40,                                             /* bMaxPacketSize */
  0x83, 0x04,                                       /* idVendor */
  0x11, 0x00,                                       /* idProduct */
  0x00, 0x01,                                       /* bcdDevice */
  0x01,                                             /* iManufacturer */
  0x02,                                             /* iProduct */
  0x03,                                             /* iSerial */
  0x01,                                             /* bNumConfigurations */
};
#define  CDC_ACM_CONFIG_DESCRIPTOR_SIZE  (75)
__ALIGN_BEGIN  const uint8_t CDC_ACM_ConfigDescriptor [75] __ALIGN_END = {
  ///////////////////////////////////////
  /// config descriptor
  ///////////////////////////////////////
  0x09,                                             /* bLength */
  USB_CONFIGURATION_DESCRIPTOR_TYPE,                /* bDescriptorType */
  0x4b, 0x00,                                       /* wTotalLength */
  0x02,                                             /* bNumInterfaces */
  0x01,                                             /* bConfigurationValue */
  0x00,                                             /* iConfiguration */
  0x80,                                             /* bmAttributes */
  0x64,                                             /* bMaxPower */
  
  ///////////////////////////////////////
  /// interface association descriptor
  ///////////////////////////////////////
  0x08,                                             /* bLength */
  USB_IAD_DESCRIPTOR_TYPE,                          /* bDescriptorType */
  0x00,                                             /* bFirstInterface */
  0x02,                                             /* bInterfaceCount */
  0x02,                                             /* bFunctionClass */
  0x02,                                             /* bFunctionSubClass */
  0x01,                                             /* bFunctionProtocol */
  0x00,                                             /* iFunction */
  
  ///////////////////////////////////////
  /// interface descriptor
  ///////////////////////////////////////
  0x09,                                             /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,                    /* bDescriptorType */
  0x00,                                             /* bInterfaceNumber */
  0x00,                                             /* bAlternateSetting */
  0x01,                                             /* bNumEndpoints */
  0x02,                                             /* bInterfaceClass */
  0x02,                                             /* bInterfaceSubClass */
  0x01,                                             /* bInterfaceProtocol */
  0x00,                                             /* iInterface */
  
  ///////////////////////////////////////
  /// cdc acm header descriptor
  ///////////////////////////////////////
  0x05,                                             /* bLength */
  0x24,                                             /* bDescriptorType */
  0x00,                                             /* bDescriptorSubtype */
  0x10, 0x01,                                       /* bcdCDC */
  
  ///////////////////////////////////////
  /// cdc acm call management descriptor
  ///////////////////////////////////////
  0x05,                                             /* bLength */
  0x24,                                             /* bDescriptorType */
  0x01,                                             /* bDescriptorSubtype */
  0x00,                                             /* bmCapabilities */
  0x01,                                             /* bDataInterface */
  
  ///////////////////////////////////////
  /// cdc acm descriptor
  ///////////////////////////////////////
  0x04,                                             /* bLength */
  0x24,                                             /* bDescriptorType */
  0x02,                                             /* bDescriptorSubtype */
  0x02,                                             /* bmCapabilities */
  
  ///////////////////////////////////////
  /// cdc acm union descriptor
  ///////////////////////////////////////
  0x05,                                             /* bLength */
  0x24,                                             /* bDescriptorType */
  0x06,                                             /* bDescriptorSubtype */
  0x00,                                             /* bMasterInterface */
  0x01,                                             /* bSlaveInterface0 */
  
  ///////////////////////////////////////
  /// endpoint descriptor
  ///////////////////////////////////////
  0x07,                                             /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x88,                                             /* bEndpointAddress */
  0x03,                                             /* bmAttributes */
  0x10, 0x00,                                       /* wMaxPacketSize */
  0x01,                                             /* bInterval */
  
  ///////////////////////////////////////
  /// interface descriptor
  ///////////////////////////////////////
  0x09,                                             /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,                    /* bDescriptorType */
  0x01,                                             /* bInterfaceNumber */
  0x00,                                             /* bAlternateSetting */
  0x02,                                             /* bNumEndpoints */
  0x0a,                                             /* bInterfaceClass */
  0x00,                                             /* bInterfaceSubClass */
  0x00,                                             /* bInterfaceProtocol */
  0x00,                                             /* iInterface */
  
  ///////////////////////////////////////
  /// endpoint descriptor
  ///////////////////////////////////////
  0x07,                                             /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x81,                                             /* bEndpointAddress */
  0x02,                                             /* bmAttributes */
  0x20, 0x00,                                       /* wMaxPacketSize */
  0x01,                                             /* bInterval */
  
  ///////////////////////////////////////
  /// endpoint descriptor
  ///////////////////////////////////////
  0x07,                                             /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,                     /* bDescriptorType */
  0x01,                                             /* bEndpointAddress */
  0x02,                                             /* bmAttributes */
  0x20, 0x00,                                       /* wMaxPacketSize */
  0x01,                                             /* bInterval */
};
#define  CDC_ACM_STRING_DESCRIPTOR0_STR   "\x09\x04"
#define  CDC_ACM_STRING_DESCRIPTOR0_SIZE  (4)
WEAK __ALIGN_BEGIN  const uint8_t CDC_ACM_StringDescriptor0 [4] __ALIGN_END = {
  0x04,                                         /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,                   /* bDescriptorType */
  0x09, 0x04,                                   /* wLangID0 */
};
#define  CDC_ACM_STRING_DESCRIPTOR1_STR   "TeenyUSB"
#define  CDC_ACM_STRING_DESCRIPTOR1_SIZE   (18)
WEAK __ALIGN_BEGIN  const uint8_t CDC_ACM_StringDescriptor1 [18] __ALIGN_END = {
  0x12,                                             /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,                       /* bDescriptorType */
  'T', 0x00,                                        /* wcChar0 */
  'e', 0x00,                                        /* wcChar1 */
  'e', 0x00,                                        /* wcChar2 */
  'n', 0x00,                                        /* wcChar3 */
  'y', 0x00,                                        /* wcChar4 */
  'U', 0x00,                                        /* wcChar5 */
  'S', 0x00,                                        /* wcChar6 */
  'B', 0x00,                                        /* wcChar7 */
};
#define  CDC_ACM_STRING_DESCRIPTOR2_STR   "TeenyUSB CDC DEMO"
#define  CDC_ACM_STRING_DESCRIPTOR2_SIZE   (36)
WEAK __ALIGN_BEGIN  const uint8_t CDC_ACM_StringDescriptor2 [36] __ALIGN_END = {
  0x24,                                             /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,                       /* bDescriptorType */
  'T', 0x00,                                        /* wcChar0 */
  'e', 0x00,                                        /* wcChar1 */
  'e', 0x00,                                        /* wcChar2 */
  'n', 0x00,                                        /* wcChar3 */
  'y', 0x00,                                        /* wcChar4 */
  'U', 0x00,                                        /* wcChar5 */
  'S', 0x00,                                        /* wcChar6 */
  'B', 0x00,                                        /* wcChar7 */
  ' ', 0x00,                                        /* wcChar8 */
  'C', 0x00,                                        /* wcChar9 */
  'D', 0x00,                                        /* wcChar10 */
  'C', 0x00,                                        /* wcChar11 */
  ' ', 0x00,                                        /* wcChar12 */
  'D', 0x00,                                        /* wcChar13 */
  'E', 0x00,                                        /* wcChar14 */
  'M', 0x00,                                        /* wcChar15 */
  'O', 0x00,                                        /* wcChar16 */
};
#define  CDC_ACM_STRING_DESCRIPTOR3_STR   "TUSB123456"
#define  CDC_ACM_STRING_DESCRIPTOR3_SIZE   (22)
WEAK __ALIGN_BEGIN  const uint8_t CDC_ACM_StringDescriptor3 [22] __ALIGN_END = {
  0x16,                                             /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,                       /* bDescriptorType */
  'T', 0x00,                                        /* wcChar0 */
  'U', 0x00,                                        /* wcChar1 */
  'S', 0x00,                                        /* wcChar2 */
  'B', 0x00,                                        /* wcChar3 */
  '1', 0x00,                                        /* wcChar4 */
  '2', 0x00,                                        /* wcChar5 */
  '3', 0x00,                                        /* wcChar6 */
  '4', 0x00,                                        /* wcChar7 */
  '5', 0x00,                                        /* wcChar8 */
  '6', 0x00,                                        /* wcChar9 */
};
#define CDC_ACM_STRING_COUNT    (4)
const uint8_t* const CDC_ACM_StringDescriptors[4] = {
  CDC_ACM_StringDescriptor0,
  CDC_ACM_StringDescriptor1,
  CDC_ACM_StringDescriptor2,
  CDC_ACM_StringDescriptor3,
};


CDC_ACM_TXEP_MAX_SIZE
CDC_ACM_RXEP_MAX_SIZE
//  Device descriptors
const tusb_descriptors CDC_ACM_descriptors = {
  .device = CDC_ACM_DeviceDescriptor,
  .config = CDC_ACM_ConfigDescriptor,
  .strings = CDC_ACM_StringDescriptors,
  .string_cnt = CDC_ACM_STRING_COUNT,
#if defined(HAS_WCID)
#if defined(CDC_ACM_WCID_DESCRIPTOR_SIZE)
  .wcid_desc = CDC_ACM_WCIDDescriptor,
#else
  .wcid_desc = 0,
#endif // CDC_ACM_WCID_DESCRIPTOR_SIZE)

#if defined(CDC_ACM_WCID_PROPERTIES_SIZE)
  .wcid_properties = CDC_ACM_WCIDProperties,
#else
  .wcid_properties = 0,
#endif // CDC_ACM_WCID_PROPERTIES_SIZE

#endif // HAS_WCID

#if defined(HAS_WCID_20)
#if defined(CDC_ACM_WCID_BOS_SIZE)
  .wcid_bos = CDC_ACM_WCIDBOS,
#else
  .wcid_bos = 0,  
#endif // CDC_ACM_WCID_BOS_SIZE)

#if defined(CDC_ACM_WCID_DESC_SET_SIZE)
  .wcid_desc_set = CDC_ACM_WCIDDescriptorSet,
#else
  .wcid_desc_set = 0,  
#endif // CDC_ACM_WCID_DESC_SET_SIZE


#endif // HAS_WCID_20
};
