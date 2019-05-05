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

//only for ep != 0
static bool set_tx_fifo(uint8_t ep, uint16_t epsize)
{
	const uint32_t DIEPTXF0_HNPTXFSIZ = OTG->DIEPTXF0_HNPTXFSIZ;
	const uint32_t TX0FD  = DIEPTXF0_HNPTXFSIZ >> 16;
	const uint32_t TX0FSA = DIEPTXF0_HNPTXFSIZ & 0x0000FFFF;

    //first fifo address availible for ep1
    uint32_t fsa = TX0FD + TX0FSA;

    //next free address
    for (int i = 0; i < (ep - 1); i++)
    {
        const uint32_t DIEPTXF = OTG->DIEPTXF[i];
        const uint32_t INEPTXFD = DIEPTXF >> 16;
        const uint32_t INEPTXSA = DIEPTXF & 0x0000FFFF;

        fsa += INEPTXFD;
    }

    /* calculating requited TX fifo size */
    /* getting in 32 bit terms */
    epsize = (epsize + 0x03) >> 2;
    /* it must be 16 32-bit words minimum */
    if (epsize < 0x10) epsize = 0x10;
    /* checking for the available fifo */
    if ((fsa + epsize) > MAX_FIFO_SZ) return false;
    /* programming fifo register */
    fsa |= (epsize << 16);
    OTG->DIEPTXF[ep - 1] = fsa;
    return true;
}
static bool ep_config(uint8_t ep, uint8_t eptype, uint16_t epsize)
{
	if (ep == 0) {
        /* configureing control endpoint EP0 */
        uint32_t mpsize;
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
        /* EP0 TX FIFO size is setted on init level */
        /* enabling RX and TX interrupts from EP0 */
        OTGD->DAINTMSK |= 0x00010001;
        /* setting up EP0 TX and RX registers */

        //DIEPTSIZ0
        EPIN(ep)->DIEPTSIZ  = (1 << 19) | epsize;
        EPIN(ep)->DIEPCTL = USB_OTG_DIEPCTL_SNAK | mpsize;

        /* 1 setup packet, 1 packets total */
        //DOEPTSIZ0
        EPOUT(ep)->DOEPTSIZ = (1 << 29) | (1 << 19) | epsize;

        //DOEPCTL0
        EPOUT(ep)->DOEPCTL = USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK | mpsize;
        return true;
    }
    if (ep & 0x80) {
        ep &= 0x7F;
        USB_OTG_INEndpointTypeDef* epi = EPIN(ep);
        /* configuring TX endpoint */
        /* setting up TX fifo and size register */
        if ((eptype == USB_EPTYPE_ISOCHRONUS) ||
            (eptype == (USB_EPTYPE_BULK | USB_EPTYPE_DBLBUF))) {
            if (!set_tx_fifo(ep, epsize << 1)) return false;
        } else {
            if (!set_tx_fifo(ep, epsize)) return false;
        }
        /* enabling EP TX interrupt */
        OTGD->DAINTMSK |= (0x0001UL << ep);
        /* setting up TX control register*/
        switch (eptype) {
        case USB_EPTYPE_ISOCHRONUS:
            epi->DIEPCTL = USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK |
                           (0x01 << 18) | USB_OTG_DIEPCTL_USBAEP |
                           USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                           (ep << 22) | epsize;
            break;
        case USB_EPTYPE_BULK:
        case USB_EPTYPE_BULK | USB_EPTYPE_DBLBUF:
            epi->DIEPCTL = USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_USBAEP |
                            (0x02 << 18) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                            (ep << 22) | epsize;
            break;
        default:
            epi->DIEPCTL = USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_USBAEP |
                            (0x03 << 18) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                            (ep << 22) | epsize;
            break;
        }
    } else {
        /* configuring RX endpoint */
        USB_OTG_OUTEndpointTypeDef* epo = EPOUT(ep);
        /* setting up RX control register */
        switch (eptype) {
        case USB_EPTYPE_ISOCHRONUS:
            epo->DOEPCTL = USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_CNAK |
                           USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP |
                           (0x01 << 18) | epsize;
            break;
        case USB_EPTYPE_BULK | USB_EPTYPE_DBLBUF:
        case USB_EPTYPE_BULK:
            epo->DOEPCTL = USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_CNAK |
                           USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP |
                           (0x02 << 18) | epsize;
            break;
        default:
            epo->DOEPCTL = USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_CNAK |
                           USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP |
                           (0x03 << 18) | epsize;
            break;
        }
    }
    return true;
}
static void ep_deconfig(uint8_t ep)
{
    ep &= 0x7F;
    volatile USB_OTG_INEndpointTypeDef*  epi = EPIN(ep);
    volatile USB_OTG_OUTEndpointTypeDef* epo = EPOUT(ep);
    /* deconfiguring TX part */
    /* disable interrupt */
    OTGD->DAINTMSK &= ~(0x00010001 << ep);
    /* decativating endpoint */
    _BCL(epi->DIEPCTL, USB_OTG_DIEPCTL_USBAEP);
    /* flushing FIFO */
    Flush_TX(ep);
    /* disabling endpoint */
    if ((epi->DIEPCTL & USB_OTG_DIEPCTL_EPENA) && (ep != 0)) {
        epi->DIEPCTL = USB_OTG_DIEPCTL_EPDIS;
    }
    /* clean EP interrupts */
    epi->DIEPINT = 0xFF;
    /* deconfiguring TX FIFO */
    if (ep > 0) {
        OTG->DIEPTXF[ep-1] = 0x02000200 + 0x200 * ep;
    }
    /* deconfigureing RX part */
    _BCL(epo->DOEPCTL, USB_OTG_DOEPCTL_USBAEP);
    if ((epo->DOEPCTL & USB_OTG_DOEPCTL_EPENA) && (ep != 0)) {
        epo->DOEPCTL = USB_OTG_DOEPCTL_EPDIS;
    }
    epo->DOEPINT = 0xFF;
}
static int32_t ep_read(uint8_t ep, void* buf, uint16_t blen)
{
    int32_t len;
    volatile uint32_t *fifo = EPFIFO(0);
    /* no data in RX FIFO */
    if (!(OTG->GINTSTS & USB_OTG_GINTSTS_RXFLVL)) return -1;
    ep &= 0x7F;
    if ((OTG->GRXSTSR & USB_OTG_GRXSTSP_EPNUM) != ep) return -1;
    /* pop data from fifo */
    len = _FLD2VAL(USB_OTG_GRXSTSP_BCNT, OTG->GRXSTSP);
    for (unsigned i = 0; i < len; i +=4) {
        uint32_t _t = *fifo;
        if (blen >= 4) {
            *(__attribute__((packed))uint32_t*)buf = _t;
            blen -= 4;
            buf += 4;
        } else {
            while (blen){
                *(uint8_t*)buf++ = 0xFF & _t;
                _t >>= 8;
                blen --;
            }
        }
    }
    return len;
}
static int32_t ep_write(uint8_t ep, void *buf, uint16_t blen)
{
    ep &= 0x7F;
    volatile uint32_t* _fifo = EPFIFO(ep);
    volatile USB_OTG_INEndpointTypeDef* epi = EPIN(ep);
    /* transfer data size in 32-bit words */
    uint32_t  _len = (blen + 3) >> 2;
    /* no enough space in TX fifo */
    if (_len > _FLD2VAL(USB_OTG_DTXFSTS_INEPTFSAV, epi->DTXFSTS)) return -1;
    if (ep != 0 && epi->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
        return -1;
    }
    _BMD(epi->DIEPTSIZ,
         USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_MULCNT | USB_OTG_DIEPTSIZ_XFRSIZ,
         _VAL2FLD(USB_OTG_DIEPTSIZ_PKTCNT, 1) | _VAL2FLD(USB_OTG_DIEPTSIZ_MULCNT, 1 ) | _VAL2FLD(USB_OTG_DIEPTSIZ_XFRSIZ, blen));
    _BMD(epi->DIEPCTL, USB_OTG_DIEPCTL_STALL, USB_OTG_DIEPCTL_CNAK);
    _BST(epi->DIEPCTL, USB_OTG_DIEPCTL_EPENA);
    while (_len--) {
        *_fifo = *(__attribute__((packed)) uint32_t*)buf;
        buf += 4;
    }
    return blen;
}
static uint16_t get_frame(void)
{
	return _FLD2VAL(USB_OTG_DSTS_FNSOF, OTGD->DSTS);
}
static void evt_poll(usbd_device *dev, usbd_evt_callback callback)
{
    uint32_t evt;
    uint32_t ep = 0;
    while (1) {
        uint32_t _t = OTG->GINTSTS;
        /* bus RESET event */
        if (_t & USB_OTG_GINTSTS_USBRST) {
            OTG->GINTSTS = USB_OTG_GINTSTS_USBRST | USB_OTG_GINTSTS_RSTDET;
            for (uint8_t i = 0; i < MAX_EP; i++ ) {
                ep_deconfig(i);
            }
            Flush_RX();
            continue;
        } else if (_t & USB_OTG_GINTSTS_ENUMDNE) {
            OTG->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
            evt = usbd_evt_reset;
        } else if (_t & USB_OTG_GINTSTS_IEPINT) {
            for (;; ep++) {
                USB_OTG_INEndpointTypeDef* epi = EPIN(ep);
                if (ep >= MAX_EP) return;
                if (epi->DIEPINT & USB_OTG_DIEPINT_XFRC) {
                    _BST(epi->DIEPINT, USB_OTG_DIEPINT_XFRC);
                    evt = usbd_evt_eptx;
                    ep |= 0x80;
                    break;
                }
            }
        } else if (_t & USB_OTG_GINTSTS_RXFLVL) {
            _t = OTG->GRXSTSR;
            ep = _t & USB_OTG_GRXSTSP_EPNUM;
            switch (_FLD2VAL(USB_OTG_GRXSTSP_PKTSTS, _t)) {
            case 0x02:  /* OUT recieved */
                evt = usbd_evt_eprx;
                break;
            case 0x06:  /* SETUP recieved */
                /* flushing TX if something stuck in control endpoint */
                if (EPIN(ep)->DIEPTSIZ & USB_OTG_DIEPTSIZ_PKTCNT) {
                    Flush_TX(ep);
                }
                evt = usbd_evt_epsetup;
                break;
            case 0x03:  /* OUT completed */
            case 0x04:  /* SETUP completed */
                _BST(EPOUT(ep)->DOEPCTL, USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
            default:
                /* pop GRXSTSP */
                OTG->GRXSTSP;
                continue;
            }
#if !defined(USBD_SOF_DISABLED)
        } else if (_t & USB_OTG_GINTSTS_SOF) {
            OTG->GINTSTS = USB_OTG_GINTSTS_SOF;
            evt = usbd_evt_sof;
#endif
        } else if (_t & USB_OTG_GINTSTS_USBSUSP) {
            evt = usbd_evt_susp;
            OTG->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
        } else if(_t & USB_OTG_GINTSTS_ESUSP) {
			OTG->GINTSTS = USB_OTG_GINTSTS_ESUSP;
			continue;
    	} else if (_t & USB_OTG_GINTSTS_WKUINT) {
            OTG->GINTSTS = USB_OTG_GINTSTS_WKUINT;
            evt = usbd_evt_wkup;
        } else {
            /* no more supported events */
            return;
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