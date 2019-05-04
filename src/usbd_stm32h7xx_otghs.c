/* This file is the part of the Lightweight USB device Stack for STM32 microcontrollers
 *
 * Copyright ©2019 Jacob Schloss <jacob[at]schloss[dot]com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32.h"
#include "usb.h"

#define MAX_EP          x
#define MAX_RX_PACKET   x
#define MAX_CONTROL_EP  x
#define MAX_FIFO_SZ     1024  /*4 * 1024B*/

#define RX_FIFO_SZ      x

static USB_OTG_GlobalTypeDef * const OTG  = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_GLOBAL_BASE);
static USB_OTG_DeviceTypeDef * const OTGD = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
static volatile uint32_t * const OTGPCTL  = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE);

static inline volatile uint32_t* EPFIFO(uint8_t ep);
static inline USB_OTG_INEndpointTypeDef* EPIN(uint8_t ep);
static inline USB_OTG_OUTEndpointTypeDef* EPOUT(uint8_t ep);

static void Flush_RX(void);
static void Flush_TX(uint8_t ep);

static uint32_t getinfo(void);
static void ep_setstall(uint8_t ep, bool stall);
static bool ep_isstalled(uint8_t ep);
static void enable(bool enable);
static uint8_t connect(bool connect);
static void setaddr (uint8_t addr);
static bool set_tx_fifo(uint8_t ep, uint16_t epsize);
static bool ep_config(uint8_t ep, uint8_t eptype, uint16_t epsize);
static void ep_deconfig(uint8_t ep);
static int32_t ep_read(uint8_t ep, void* buf, uint16_t blen);
static int32_t ep_write(uint8_t ep, void *buf, uint16_t blen);
static uint16_t get_frame(void);
static void evt_poll(usbd_device *dev, usbd_evt_callback callback);
static uint16_t get_serialno_desc(void *buffer);

__attribute__((externally_visible)) const struct usbd_driver usbd_otghs = {
    getinfo,
    enable,
    connect,
    setaddr,
    ep_config,
    ep_deconfig,
    ep_read,
    ep_write,
    ep_setstall,
    ep_isstalled,
    evt_poll,
    get_frame,
    get_serialno_desc,
};