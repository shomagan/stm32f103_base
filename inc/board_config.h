/*       
 *         _______                    _    _  _____ ____  
 *        |__   __|                  | |  | |/ ____|  _ \ 
 *           | | ___  ___ _ __  _   _| |  | | (___ | |_) |
 *           | |/ _ \/ _ \ '_ \| | | | |  | |\___ \|  _ < 
 *           | |  __/  __/ | | | |_| | |__| |____) | |_) |
 *           |_|\___|\___|_| |_|\__, |\____/|_____/|____/ 
 *                               __/ |                    
 *                              |___/                     
 *
 * TeenyUSB - light weight usb stack for STM32 micro controllers
 * 
 * Copyright (c) 2019 XToolBox  - admin@xtoolbox.org
 *                         www.tusb.org
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

/* Use external phy for high speed core */
// #define  OTG_HS_EXTERNAL_PHY

/* Use embedded phy for high speed core */
// #define  OTG_HS_EMBEDDED_PHY

/* Use embedded phy for full speed core */ 
// #define  OTG_FS_EMBEDDED_PHY

/* Enable DMA for High speed phy */
// #define  OTG_HS_ENABLE_DMA

/* Support for other speed config and device qualifier descriptor */
#define USB_USE 1
#define RTC_USE 1
#define ADC_USE 0
#define UART_USE 1
#define CRC_USE 1
#define USE_GPIO_PORT_A 1
#define USE_GPIO_PORT_B 1
#define USE_GPIO_PORT_C 1
#define USE_GPIO_PORT_D 1
#define  SUPPORT_OTHER_SPEED
#define IWDG_USE 1

/* Setup descriptor buffer size, used for other speed config and DMA */
#define  DESCRIPTOR_BUFFER_SIZE  256

/* USB core ID used in test app, 0 - FS core, 1 - HS core */
#define  USB_CORE_ID_FS             0
//#define  USB_CORE_ID_HS             1

#define  TEST_APP_USB_CORE          USB_CORE_ID_FS

/* STM32F103RE flash size */
#define  FLASH_SIZE  (64*1024)

void flash_write(uint32_t addr, const uint8_t* buf, uint32_t size);

#endif

