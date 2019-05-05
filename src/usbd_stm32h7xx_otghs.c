/* This file is the part of the Lightweight USB device Stack for STM32 microcontrollers
 *
 * Copyright (c) 2016 Dmitry Filimonchuk <dmitrystu[at]gmail[dot]com>
 * Copyright (c) 2019 Jacob Schloss <jacob[at]schloss[dot]com>
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

#define MAX_EP          8
#define MAX_RX_PACKET   128
#define MAX_CONTROL_EP  1
#define MAX_FIFO_SZ     1024  /*4 * 1024B*/

//10 for setup packets
//1 global out nak
//2*pak/4+1
//one location per out endpoint
#define RX_FIFO_SZ      (10 + 1 + (2*MAX_RX_PACKET/4 + 1) + 2*MAX_EP)

static volatile USB_OTG_GlobalTypeDef* const OTG  = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_GLOBAL_BASE);
static volatile USB_OTG_DeviceTypeDef* const OTGD = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE);
static volatile uint32_t * const OTGPCTL  = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE);

static inline volatile uint32_t* EPFIFO(uint8_t ep)
{
	return (volatile uint32_t*)(USB1_OTG_HS_PERIPH_BASE + USB_OTG_FIFO_BASE + (ep * USB_OTG_FIFO_SIZE));
}
static inline volatile USB_OTG_INEndpointTypeDef* EPIN(uint8_t ep)
{
	return (volatile USB_OTG_INEndpointTypeDef*)(USB1_OTG_HS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (ep * USB_OTG_EP_REG_SIZE));
}
static inline volatile USB_OTG_OUTEndpointTypeDef* EPOUT(uint8_t ep)
{
	return (volatile USB_OTG_OUTEndpointTypeDef*)(USB1_OTG_HS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (ep * USB_OTG_EP_REG_SIZE));
}

inline static void Flush_RX(void) {
    _BST(OTG->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);
    _WBC(OTG->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);
}

inline static void Flush_TX(uint8_t ep) {
    _BMD(OTG->GRSTCTL, USB_OTG_GRSTCTL_TXFNUM,
         _VAL2FLD(USB_OTG_GRSTCTL_TXFNUM, ep) | USB_OTG_GRSTCTL_TXFFLSH);
    _WBC(OTG->GRSTCTL, USB_OTG_GRSTCTL_TXFFLSH);
}

static uint32_t getinfo(void)
{
	if((RCC->AHB1ENR & RCC_AHB1ENR_USB1OTGHSEN) == 0)
	{
		return USBD_HW_ADDRFST;
	}

	if(_FLD2VAL(USB_OTG_DSTS_ENUMSPD, OTGD->DSTS) == 0x00)
	{
		return USBD_HW_ADDRFST | USBD_HW_ENABLED | USBD_HW_SPEED_HS;
	}

	if(_FLD2VAL(USB_OTG_DSTS_ENUMSPD, OTGD->DSTS) == 0x01)
	{
		return USBD_HW_ADDRFST | USBD_HW_ENABLED | USBD_HW_SPEED_FS;
	}

	if(_FLD2VAL(USB_OTG_DSTS_ENUMSPD, OTGD->DSTS) == 0x03)
	{
		return USBD_HW_ADDRFST | USBD_HW_ENABLED | USBD_HW_SPEED_FS;
	}

	return USBD_HW_ADDRFST | USBD_HW_ENABLED | USBD_HW_SPEED_NC;
}
static void ep_setstall(uint8_t ep, bool stall)
{
    if (ep & 0x80) {
        ep &= 0x7F;
        uint32_t _t = EPIN(ep)->DIEPCTL;
        if (_t & USB_OTG_DIEPCTL_USBAEP) {
            if (stall) {
                _BST(_t, USB_OTG_DIEPCTL_STALL);
            } else {
                _BMD(_t, USB_OTG_DIEPCTL_STALL,
                     USB_OTG_DIEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_SNAK);
            }
            EPIN(ep)->DIEPCTL = _t;
        }
    } else {
        uint32_t _t = EPOUT(ep)->DOEPCTL;
        if (_t & USB_OTG_DOEPCTL_USBAEP) {
            if (stall) {
                _BST(_t, USB_OTG_DOEPCTL_STALL);
            } else {
                _BMD(_t, USB_OTG_DOEPCTL_STALL,
                     USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_CNAK);
            }
            EPOUT(ep)->DOEPCTL = _t;
        }
    }
}
static bool ep_isstalled(uint8_t ep)
{
	const uint8_t ep_idx = ep & 0x7F;
	bool ret = false;

	if(ep & 0x80)
	{
		ret = (EPIN(ep_idx)->DIEPCTL & USB_OTG_DIEPCTL_STALL) ? true : false;
	}
	else
	{
		ret = (EPOUT(ep_idx)->DOEPCTL & USB_OTG_DOEPCTL_STALL) ? true : false;
	}

	return ret;
}

static void enable(bool enable)
{
	if(enable)
	{
		//enable usb core and ulpi clock for USB1
		_BST(RCC->AHB1ENR, RCC_AHB1ENR_USB1OTGHSEN | RCC_AHB1ENR_USB1OTGHSULPIEN);

		//wait for USB1 to be idle
		_WBS(OTG->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL);

		//No PD, no internal tranciever
		OTG->GCCFG = 0;

		_BCL(OTG->GUSBCFG, USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);

		OTG->GUSBCFG = USB_OTG_GUSBCFG_FDMOD                 | 
		               //USB_OTG_GUSBCFG_ULPIIPD               |
		               _VAL2FLD(USB_OTG_GUSBCFG_TRDT,  0x09) |
		               _VAL2FLD(USB_OTG_GUSBCFG_TOCAL, 0x01) ;


		//soft reset
		_BST(OTG->GRSTCTL, USB_OTG_GRSTCTL_CSRST);
		_WBC(OTG->GRSTCTL, USB_OTG_GRSTCTL_CSRST);

		//reset fifo assignments
		for (size_t i = 0; i < 15; i++)
		{
			OTG->DIEPTXF[i] = 0;
		}

		//start clocks, no sleep gate
		*OTGPCTL = 0;

		//USB HS
		_BMD(OTGD->DCFG, USB_OTG_DCFG_DSPD, _VAL2FLD(USB_OTG_DCFG_DSPD, 0x00));
        OTGD->DCFG |= USB_OTG_DCFG_NZLSOHSK;

		// _BMD(OTGD->DCFG, USB_OTG_DCFG_DSPD, _VAL2FLD(USB_OTG_DCFG_DSPD, 0x01));

		Flush_TX(0x10);//Flush all tx fifo
		Flush_RX();
	
		OTGD->DIEPMSK = 0U;
		OTGD->DOEPMSK = 0U;
		OTGD->DAINTMSK = 0U;

		//soft disconnect
		_BST(OTGD->DCTL, USB_OTG_DCTL_SDIS);

		for(size_t i = 0; i < MAX_EP; i++)
		{
			if(EPIN(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
			{
				if(i == 0)
				{
					EPIN(i)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
				}
				else
				{
					EPIN(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;	
				}
			}
			else
			{
				EPIN(i)->DIEPCTL = 0;
			}

			EPIN(i)->DIEPTSIZ = 0;
			EPIN(i)->DIEPINT = 0x0000FFFF;
		}

		for(size_t i = 0; i < MAX_EP; i++)
		{
			if(EPOUT(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
			{
				if(i == 0)
				{
					EPOUT(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
				}
				else
				{
					EPOUT(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;	
				}
			}
			else
			{
				EPOUT(i)->DOEPCTL = 0;
			}

			EPOUT(i)->DOEPTSIZ = 0;
			EPOUT(i)->DOEPINT = 0x0000FFFF;
		}

		//rx fifo
        OTG->GRXFSIZ = RX_FIFO_SZ;
		//ep0 tx fifo 64 byte
        OTG->DIEPTXF0_HNPTXFSIZ = (0x10 << 16) | RX_FIFO_SZ;
        
        //ep tx interrupt
        OTGD->DIEPMSK = USB_OTG_DIEPMSK_XFRCM;

		//clear core interrupt
		OTG->GINTMSK = 0U;
		OTG->GINTSTS = 0xFFFFFFFF;

        //core interrupt
        OTG->GINTMSK  = USB_OTG_GINTMSK_USBRST   |
        				USB_OTG_GINTMSK_ENUMDNEM |
                        USB_OTG_GINTMSK_USBSUSPM |
                        USB_OTG_GINTMSK_ESUSPM   |
#if !defined(USBD_SOF_DISABLED)
                        //USB_OTG_GINTMSK_SOFM    
#endif
                        USB_OTG_GINTMSK_WUIM     |
                        USB_OTG_GINTMSK_IEPINT   |
                        USB_OTG_GINTMSK_RXFLVLM  ;
                        ;


		//turn on global interrupt
		_BST(OTG->GAHBCFG, USB_OTG_GAHBCFG_GINT);
	}
	else
	{
		if(RCC->AHB1ENR & (RCC_AHB1ENR_USB1OTGHSEN | RCC_AHB1ENR_USB1OTGHSULPIEN))
		{
			//reset USB1
			_BST(RCC->AHB1RSTR, RCC_AHB1RSTR_USB1OTGHSRST);
			_BCL(RCC->AHB1RSTR, RCC_AHB1RSTR_USB1OTGHSRST);

			//gate clocks
			_BCL(RCC->AHB1ENR, RCC_AHB1ENR_USB1OTGHSEN | RCC_AHB1ENR_USB1OTGHSULPIEN);
		}
	}
}

static uint8_t connect(bool connect)
{
	if(connect)
	{
		_BCL(OTGD->DCTL, USB_OTG_DCTL_SDIS);
	}
	else
	{
		_BST(OTGD->DCTL, USB_OTG_DCTL_SDIS);
	}

	return usbd_lane_unk;
}
static void setaddr(uint8_t addr)
{
	OTGD->DCFG |= _VAL2FLD(USB_OTG_DCFG_DAD, addr);
}

static bool set_tx_fifo(uint8_t ep, uint16_t epsize)
{
    const uint32_t DIEPTXF0 = OTG->DIEPTXF0_HNPTXFSIZ;
    const uint32_t TXF0FD  = DIEPTXF0 >> 16;
    const uint32_t TXF0FSA = DIEPTXF0 & 0x0000FFFF;

    for(int i = 0; i < ep; i++)
    {
        const uint32_t DIEPTXF = OTG->DIEPTXF[i];

        const uint32_t FD  = DIEPTXF >> 16;
        const uint32_t FSA = DIEPTXF & 0x0000FFFF;
    }
}
static bool ep_config(uint8_t ep, uint8_t eptype, uint16_t epsize)
{
    if(ep == 0)
    {
        //ep0 rx & tx
        OTGD->DAINTMSK |= 0x00010001;

        OTGD->DOEPMSK |= USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM;
        OTGD->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_TOM;

        //set rx size
        OTG->GRXFSIZ = RX_FIFO_SZ;

        //set ep0 start and size
        OTG->DIEPTXF0_HNPTXFSIZ = _VAL2FLD(USB_OTG_TX0FD, epsize) | _VAL2FLD(USB_OTG_TX0FSA, RX_FIFO_SZ);

        volatile USB_OTG_INEndpointTypeDef*  epin  = EPIN(0);
        volatile USB_OTG_OUTEndpointTypeDef* epout = EPOUT(0);

        uint32_t mpsize = 0;
        if (epsize <= 0x08) {
            epsize = 0x08;
            mpsize = 0x03;
        } else if (epsize <= 0x10) {
            epsize = 0x10;
            mpsize = 0x02;
        } else if (epsize <= 0x20) {
            epsize = 0x20;
            mpsize = 0x01;
        } else {
            epsize = 0x40;
            mpsize = 0x00;
        }
        // epin->DIEPTSIZ
        epin->DIEPTCTL = USB_OTG_DOEPCTL_SNAK | mpsize;

        epout->DOEPTSIZ = (1 << 29) | (1 << 19) | epsize;
        epout->DOEPTCTL = USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA | mpsize;
    }
    else
    {
        if(ep & 0x80)
        {
            //tx
        }
        else
        {
            //rx
        }
    }

    return true;
}
static void ep_deconfig(uint8_t ep)
{

}
static int32_t ep_read(uint8_t ep, void* buf, uint16_t blen)
{
    volatile uint32_t *fifo = EPFIFO(0);

    /* no data in RX FIFO */
    if (!(OTG->GINTSTS & USB_OTG_GINTSTS_RXFLVL))
    {
        return -1;
    }
    ep &= 0x7F;
    //wrong ep
    if ((OTG->GRXSTSR & USB_OTG_GRXSTSP_EPNUM) != ep)
    {
        return -1;
    }

    //pop GRXSTSP
    const uint32_t GRXSTSP = OTG->GRXSTSP;
    const uint32_t len = _FLD2VAL(USB_OTG_GRXSTSP_BCNT, GRXSTSP);

    uint32_t* buf32 = (uint32_t*)buf;
    for(size_t i = 0; i < len; i += 4)
    {
        uint32_t x = *fifo;
        if((len-i) >= 4)
        {
            buf32[i] = x;
        }
        else
        {
            uint8_t* buf8 = ((uint8_t*)buf);
            for(size_t j = i; j < len; j++)
            {
                buf8[j] = x & 0x000000FF;
                x >>= 8;
            }
        }
    }

    return len;
}
static int32_t ep_write(uint8_t ep, void *buf, uint16_t blen)
{
    volatile uint32_t* fifo = EPFIFO(ep);
    volatile USB_OTG_INEndpointTypeDef* epi = EPIN(ep);

    const uint32_t len32 = (blen + 3) / 4;

    if(len32 > _FLD2VAL(USB_OTG_DTXFSTS_INEPTFSAV, epi->DTXFSTS))
    {
        return -1;
    }
    if((ep != 0) && (epi->DIEPCTL & USB_OTG_DIEPCTL_EPENA))
    {
        return -1;
    }

    _BMD(epi->DIEPTSIZ,
         USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_MULCNT | USB_OTG_DIEPTSIZ_XFRSIZ,
         _VAL2FLD(USB_OTG_DIEPTSIZ_PKTCNT, 1) | _VAL2FLD(USB_OTG_DIEPTSIZ_MULCNT, 1 ) | _VAL2FLD(USB_OTG_DIEPTSIZ_XFRSIZ, blen));
    _BMD(epi->DIEPCTL, USB_OTG_DIEPCTL_STALL, USB_OTG_DOEPCTL_CNAK);
    _BST(epi->DIEPCTL, USB_OTG_DOEPCTL_EPENA);

    //TODO: this may over-read buf by up to 3 bytes, which could cause an mpu or other bus exception
    uint32_t* ptr32 = (uint32_t*)buf;
    for(size_t i = 0; i < len32; i++)
    {
        *fifo = ptr32[i];
    }

    return blen;
}
static uint16_t get_frame(void)
{
	return _FLD2VAL(USB_OTG_DSTS_FNSOF, OTGD->DSTS);
}
static void evt_poll(usbd_device *dev, usbd_evt_callback callback)
{
    uint32_t evt = 0;
    uint32_t ep = 0;

    while(true)
    {    
        const uint32_t GINTSTS = OTG->GINTSTS;
        if(GINTSTS & USB_OTG_GINTSTS_USBRST)
        {
            OTG->GINTSTS = USB_OTG_GINTSTS_USBRST | USB_OTG_GINTSTS_RSTDET;
            for (uint8_t i = 0; i < MAX_EP; i++ )
            {
                ep_deconfig(i);
            }
            Flush_RX();
            continue;
        }
        else if(GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
        {
            OTG->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
            evt = usbd_evt_reset;
        }
        else if(USB_OTG_GINTSTS_IEPINT)
        {
            for(size_t i = 0; i <= MAX_EP; i++)
            {
                volatile USB_OTG_INEndpointTypeDef* epi = EPIN(ep);
                if (epi->DIEPINT & USB_OTG_DIEPINT_XFRC)
                {
                    _BST(epi->DIEPINT, USB_OTG_DIEPINT_XFRC);
                    evt = usbd_evt_eptx;
                    ep |= 0x80;
                    break;
                }
            }
            return;
        }
        else if(USB_OTG_GINTSTS_RXFLVL)
        {
            const uint32_t GRXSTSR = OTG->GRXSTSR;
            ep = GRXSTSR & USB_OTG_GRXSTSP_EPNUM;
            switch(_FLD2VAL(USB_OTG_GRXSTSP_PKTSTS, GRXSTSR))
            {
                case 0x02: // OUT rx
                {
                    evt = usbd_evt_eprx;
                    break;
                }
                case 0x06: // SETUP rx
                {
                    evt = usbd_evt_epsetup;
                    break;
                }
                case 0x03: //OUT Complete
                case 0x04: //SETUP Complete
                {
                    _BST(EPOUT(ep)->DOEPCTL, USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
                }
                default:
                {
                    //force a read
                    volatile uint32_t GRXSTSR = OTG->GRXSTSR;
                    continue;
                }
            }
        }
#if !defined(USBD_SOF_DISABLED)
        else if(USB_OTG_GINTSTS_SOF)
        {
            OTG->GINTSTS = USB_OTG_GINTSTS_SOF;
            evt = usbd_evt_sof;
        }
#endif
        else if(USB_OTG_GINTSTS_USBSUSP)
        {
            OTG->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
            evt = usbd_evt_susp;
        }
        else if(USB_OTG_GINTSTS_ESUSP)
        {
            OTG->GINTSTS = USB_OTG_GINTSTS_ESUSP;
            continue;
        }
        else if(USB_OTG_GINTSTS_WKUINT)
        {
            OTG->GINTSTS = USB_OTG_GINTSTS_WKUINT;
            evt = usbd_evt_wkup;
        }
        
        return callback(dev, evt, ep);
    }
}
static uint16_t get_serialno_desc(void *buffer)
{
	volatile uint32_t* uid_ptr = (volatile uint32_t*)(UID_BASE);

	const uint32_t uid0 = uid_ptr[0];
	const uint32_t uid1 = uid_ptr[1];
	const uint32_t uid2 = uid_ptr[2];

	struct usb_string_descriptor *dsc = buffer;

	for(int i = 0; i < 24; i++)
	{

	}

	dsc->bDescriptorType = USB_DTYPE_STRING;
	dsc->bLength = 24;

	return dsc->bLength;
}

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
    get_serialno_desc
};