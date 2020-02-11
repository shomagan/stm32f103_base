/////////////////////////////////////////////////////////////
//// Auto generate by TeenyDT, http://dt.tusb.org
/////////////////////////////////////////////////////////////
#ifndef __CDC_ACM_TEENY_USB_INIT_H__
#define __CDC_ACM_TEENY_USB_INIT_H__
// forward declare the tusb_descriptors struct
typedef struct _tusb_descriptors tusb_descriptors;

#define CDC_ACM_VID                                            0x0483
#define CDC_ACM_PID                                            0x0011
#define CDC_ACM_STRING_COUNT                                   (4)

// device.bmAttributes & 0x40   USB_CONFIG_SELF_POWERED
// device.bmAttributes & 0x20   USB_CONFIG_REMOTE_WAKEUP
#define CDC_ACM_DEV_STATUS                                    (0 | 0)


// Endpoint usage:
#define CDC_ACM_MAX_EP                                         (1)
#define CDC_ACM_EP_NUM                                         (CDC_ACM_MAX_EP + 1)

///////////////////////////////////////////////
//// Endpoint define for STM32 FS Core
///////////////////////////////////////////////

#ifdef CDC_ACM_BTABLE_ADDRESS
#undef CDC_ACM_BTABLE_ADDRESS
#endif
#define CDC_ACM_BTABLE_ADDRESS                                 (0)
#define CDC_ACM_EP_BUF_DESC_TABLE_SIZE                         (8)
// PMA buffer reserved for buffer description table
#define CDC_ACM_USB_BUF_START                                  (CDC_ACM_EP_BUF_DESC_TABLE_SIZE * CDC_ACM_EP_NUM)

// EndPoints 0 defines
#define CDC_ACM_EP0_RX_SIZE                                    (64)
#define CDC_ACM_EP0_RX_ADDR                                    (CDC_ACM_USB_BUF_START + (0))
#define CDC_ACM_EP0_TX_SIZE                                    (64)
#define CDC_ACM_EP0_TX_ADDR                                    (CDC_ACM_USB_BUF_START + (64))
#define CDC_ACM_EP0_RX_TYPE                                    USB_EP_CONTROL
#define CDC_ACM_EP0_TX_TYPE                                    USB_EP_CONTROL

#define CDC_ACM_EP0_TYPE                                       USB_EP_CONTROL
#define CDC_ACM_EP0_TX0_ADDR                                   (CDC_ACM_USB_BUF_START + (0))
#define CDC_ACM_EP0_TX1_ADDR                                   (CDC_ACM_USB_BUF_START + (64))
#define CDC_ACM_EP0_RX0_ADDR                                   (CDC_ACM_USB_BUF_START + (0))
#define CDC_ACM_EP0_RX1_ADDR                                   (CDC_ACM_USB_BUF_START + (64))

// EndPoints 1 defines
#define CDC_ACM_EP1_RX_SIZE                                    (32)
#define CDC_ACM_EP1_RX_ADDR                                    (CDC_ACM_USB_BUF_START + (128))
#define CDC_ACM_EP1_TX_SIZE                                    (32)
#define CDC_ACM_EP1_TX_ADDR                                    (CDC_ACM_USB_BUF_START + (160))
#define CDC_ACM_EP1_RX_TYPE                                    USB_EP_BULK
#define CDC_ACM_EP1_TX_TYPE                                    USB_EP_BULK

#define CDC_ACM_EP1_TYPE                                       USB_EP_BULK
#define CDC_ACM_EP1_TX0_ADDR                                   (CDC_ACM_USB_BUF_START + (128))
#define CDC_ACM_EP1_TX1_ADDR                                   (CDC_ACM_USB_BUF_START + (160))
#define CDC_ACM_EP1_RX0_ADDR                                   (CDC_ACM_USB_BUF_START + (128))
#define CDC_ACM_EP1_RX1_ADDR                                   (CDC_ACM_USB_BUF_START + (160))


// EndPoint max packed sizes
extern const uint8_t CDC_ACM_txEpMaxSize[];
#define CDC_ACM_TXEP_MAX_SIZE                                  \
const uint8_t CDC_ACM_txEpMaxSize[] = \
{ CDC_ACM_EP0_TX_SIZE, CDC_ACM_EP1_TX_SIZE,  };
extern const uint8_t CDC_ACM_rxEpMaxSize[];
#define CDC_ACM_RXEP_MAX_SIZE                                  \
const uint8_t CDC_ACM_rxEpMaxSize[] = \
{ CDC_ACM_EP0_RX_SIZE, CDC_ACM_EP1_RX_SIZE,  };

// EndPoints init function for USB FS core
#define CDC_ACM_TUSB_INIT_EP_FS(dev) \
  do{\
    /* Init ep0 */ \
    INIT_EP_BiDirection(dev, PCD_ENDP0, CDC_ACM_EP0_TYPE);  \
    SET_TX_ADDR(dev, PCD_ENDP0, CDC_ACM_EP0_TX_ADDR);  \
    SET_RX_ADDR(dev, PCD_ENDP0, CDC_ACM_EP0_RX_ADDR);  \
    SET_RX_CNT(dev, PCD_ENDP0, CDC_ACM_EP0_RX_SIZE);  \
    /* Init ep1 */ \
    INIT_EP_BiDirection(dev, PCD_ENDP1, CDC_ACM_EP1_TYPE);  \
    SET_TX_ADDR(dev, PCD_ENDP1, CDC_ACM_EP1_TX_ADDR);  \
    SET_RX_ADDR(dev, PCD_ENDP1, CDC_ACM_EP1_RX_ADDR);  \
    SET_RX_CNT(dev, PCD_ENDP1, CDC_ACM_EP1_RX_SIZE);  \
}while(0)

///////////////////////////////////////////////
//// Endpoint define for STM32 OTG Core
///////////////////////////////////////////////
#define CDC_ACM_OTG_MAX_OUT_SIZE                               (32)
#define CDC_ACM_OTG_CONTROL_EP_NUM                             (1)
#define CDC_ACM_OTG_OUT_EP_NUM                                 (1)
// RX FIFO size / 4 > (CONTROL_EP_NUM * 5 + 8) +  (MAX_OUT_SIZE / 4 + 1) + (OUT_EP_NUM*2) + 1 = 25

///////////////////////////////////////////////
//// Endpoint define for STM32 OTG FS Core
///////////////////////////////////////////////
#define CDC_ACM_OTG_RX_FIFO_SIZE_FS                            (256)
#define CDC_ACM_OTG_RX_FIFO_ADDR_FS                            (0)
// Sum of IN ep max packet size is 96
// Remain Fifo size is 1024 in bytes, Rx Used 256 bytes 

// TODO:
// I don't know why the max count of TX fifo should <= (7 * EpMaxPacket)
// But it seems the STM32F7xx can be large than (7 * EpMaxPacket)
#define CDC_ACM_EP0_TX_FIFO_ADDR_FS                            (256)
#define CDC_ACM_EP0_TX_FIFO_SIZE_FS                            (CDC_ACM_EP0_TX_SIZE * 7)
#define CDC_ACM_EP1_TX_FIFO_ADDR_FS                            (704)
#define CDC_ACM_EP1_TX_FIFO_SIZE_FS                            (CDC_ACM_EP1_TX_SIZE * 7)
// EndPoints init function for USB OTG core
#if defined(USB_OTG_FS)
#define CDC_ACM_TUSB_INIT_EP_OTG_FS(dev) \
  do{\
    if(GetUSB(dev) == USB_OTG_FS) { \
      SET_RX_FIFO(dev, CDC_ACM_OTG_RX_FIFO_ADDR_FS, CDC_ACM_OTG_RX_FIFO_SIZE_FS);  \
      /* Init Ep0  */\
      INIT_EP_Tx(dev, PCD_ENDP0, CDC_ACM_EP0_TX_TYPE, CDC_ACM_EP0_TX_SIZE);  \
      SET_TX_FIFO(dev, PCD_ENDP0, CDC_ACM_EP0_TX_FIFO_ADDR_FS, CDC_ACM_EP0_TX_FIFO_SIZE_FS);  \
      INIT_EP_Rx(dev, PCD_ENDP0, CDC_ACM_EP0_RX_TYPE, CDC_ACM_EP0_RX_SIZE); \
      /* Init Ep1  */\
      INIT_EP_Tx(dev, PCD_ENDP1, CDC_ACM_EP1_TX_TYPE, CDC_ACM_EP1_TX_SIZE);  \
      SET_TX_FIFO(dev, PCD_ENDP1, CDC_ACM_EP1_TX_FIFO_ADDR_FS, CDC_ACM_EP1_TX_FIFO_SIZE_FS);  \
      INIT_EP_Rx(dev, PCD_ENDP1, CDC_ACM_EP1_RX_TYPE, CDC_ACM_EP1_RX_SIZE); \
    }\
  }while(0)

#else  // #if defined(USB_OTG_FS)
#define CDC_ACM_TUSB_INIT_EP_OTG_FS(dev) 
    
#endif  // #if defined(USB_OTG_FS)

///////////////////////////////////////////////
//// Endpoint define for STM32 OTG HS Core
///////////////////////////////////////////////
#define CDC_ACM_OTG_RX_FIFO_SIZE_HS                            (512)
#define CDC_ACM_OTG_RX_FIFO_ADDR_HS                            (0)
// Sum of IN ep max packet size is 96
// Remain Fifo size is 3584 in bytes, Rx Used 512 bytes 

// TODO:
// I don't know why the max count of TX fifo should <= (7 * EpMaxPacket)
// But it seems the STM32F7xx can be large than (7 * EpMaxPacket)
#define CDC_ACM_EP0_TX_FIFO_ADDR_HS                            (512)
#define CDC_ACM_EP0_TX_FIFO_SIZE_HS                            (CDC_ACM_EP0_TX_SIZE * 7)
#define CDC_ACM_EP1_TX_FIFO_ADDR_HS                            (960)
#define CDC_ACM_EP1_TX_FIFO_SIZE_HS                            (CDC_ACM_EP1_TX_SIZE * 7)
// EndPoints init function for USB OTG core
#if defined(USB_OTG_HS)
#define CDC_ACM_TUSB_INIT_EP_OTG_HS(dev) \
  do{\
    if(GetUSB(dev) == USB_OTG_HS) { \
      SET_RX_FIFO(dev, CDC_ACM_OTG_RX_FIFO_ADDR_HS, CDC_ACM_OTG_RX_FIFO_SIZE_HS);  \
      /* Init Ep0  */\
      INIT_EP_Tx(dev, PCD_ENDP0, CDC_ACM_EP0_TX_TYPE, CDC_ACM_EP0_TX_SIZE);  \
      SET_TX_FIFO(dev, PCD_ENDP0, CDC_ACM_EP0_TX_FIFO_ADDR_HS, CDC_ACM_EP0_TX_FIFO_SIZE_HS);  \
      INIT_EP_Rx(dev, PCD_ENDP0, CDC_ACM_EP0_RX_TYPE, CDC_ACM_EP0_RX_SIZE); \
      /* Init Ep1  */\
      INIT_EP_Tx(dev, PCD_ENDP1, CDC_ACM_EP1_TX_TYPE, CDC_ACM_EP1_TX_SIZE);  \
      SET_TX_FIFO(dev, PCD_ENDP1, CDC_ACM_EP1_TX_FIFO_ADDR_HS, CDC_ACM_EP1_TX_FIFO_SIZE_HS);  \
      INIT_EP_Rx(dev, PCD_ENDP1, CDC_ACM_EP1_RX_TYPE, CDC_ACM_EP1_RX_SIZE); \
    }\
  }while(0)

#else  // #if defined(USB_OTG_HS)
#define CDC_ACM_TUSB_INIT_EP_OTG_HS(dev) 
    
#endif  // #if defined(USB_OTG_HS)
#define CDC_ACM_TUSB_INIT_EP_OTG(dev) \
  do{\
    CDC_ACM_TUSB_INIT_EP_OTG_FS(dev); \
    CDC_ACM_TUSB_INIT_EP_OTG_HS(dev); \
  }while(0)


#if defined(USB)
#define CDC_ACM_TUSB_INIT_EP(dev) CDC_ACM_TUSB_INIT_EP_FS(dev)

// Teeny USB device init function for FS core
#define CDC_ACM_TUSB_INIT_DEVICE(dev) \
  do{\
    /* Init device features */       \
    memset(&dev->addr, 0, TUSB_DEVICE_SIZE);    \
    dev->status = CDC_ACM_DEV_STATUS;         \
    dev->rx_max_size = CDC_ACM_rxEpMaxSize;         \
    dev->tx_max_size = CDC_ACM_txEpMaxSize;         \
    dev->descriptors = &CDC_ACM_descriptors;         \
  }while(0)

#endif

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
#define CDC_ACM_TUSB_INIT_EP(dev) CDC_ACM_TUSB_INIT_EP_OTG(dev)

// Teeny USB device init function for OTG core
#define CDC_ACM_TUSB_INIT_DEVICE(dev) \
  do{\
    /* Init device features */       \
    memset(&dev->addr, 0, TUSB_DEVICE_SIZE);    \
    dev->status = CDC_ACM_DEV_STATUS;         \
    dev->descriptors = &CDC_ACM_descriptors;         \
  }while(0)

#endif

#define CDC_ACM_TUSB_INIT(dev) \
  do{\
    CDC_ACM_TUSB_INIT_EP(dev);   \
    CDC_ACM_TUSB_INIT_DEVICE(dev);   \
  }while(0)

// Get End Point count
#ifndef  EP_NUM
#define  EP_NUM 1
#endif
#if CDC_ACM_EP_NUM > EP_NUM
#undef   EP_NUM
#define  EP_NUM  CDC_ACM_EP_NUM
#endif

extern const uint8_t* const CDC_ACM_StringDescriptors[CDC_ACM_STRING_COUNT];
extern const tusb_descriptors CDC_ACM_descriptors;


#endif   // #ifndef __CDC_ACM_TEENY_USB_INIT_H__
