/*
  Copyright (c) 2012-2013  John Lee (j.y.lee@yeah.net)
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.

  * Neither the name of the copyright holders nor the names of
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __USB_HAL_H
#define __USB_HAL_H

#ifndef NDEBUG
#define NDEBUG
#endif

#include <assert.h>
#include <cstdint>
#include <type_traits>
#include <attribute.h>
#include <stm32/stm32f30x/usb_fs.hpp>
#include <stm32/stm32f30x/nvic.hpp>
#include <stm32/stm32f30x/rcc.hpp>
#include <stm32/stm32f30x/gpio.hpp>
#include <stm32/stm32f30x/can.hpp>
#ifdef __INLINE
#undef __INLINE
#endif

typedef enum IRQn{
    SysTick_IRQn = -1,
}IRQn_Type;
typedef struct GPIO_TypeDef GPIO_TypeDef;
/**
 * @brief Configuration of the Cortex-M4 Processor and Core Peripherals 
 */
#include "core_cm4.h"
//#include "STM32F30x/Include/stm32f30x.h"
#define __STM32F30x_H
#include "STM32_USB-FS-Device_Driver/inc/usb_regs.h"
#include "STM32F30x_StdPeriph_Driver/inc/stm32f30x_gpio.h"

#include "debug.hpp"

#define IMR_MSK (CNTR_CTRM  | CNTR_WKUPM | CNTR_SUSPM | CNTR_RESETM )


/* buffer table base address */
#define BTABLE_ADDRESS          (0x00)
///xy 编译时得到所有的ep数量。每个ep可以有in,out或2者都有
#define GET_TOTOAL_EP_COUNT()   (2)
///xy STM32使用USB时，系统时钟只能为72Mhz或是48Mhz
#define GET_SYS_CLOCK()         (72000000ul)

#ifdef _GetBTABLE
#undef _GetBTABLE
#define _GetBTABLE()   BTABLE_ADDRESS
#endif

using namespace sfr::usb_fs;
using namespace sfr::nvic;
using namespace sfr::gpio;
using namespace sfr::rcc;
using namespace sfr::can;

namespace usbd {
	struct __attribute__((aligned(2), packed)) setup_pkt_t {
		__INLINE setup_pkt_t(uint8_t type, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
		{
			pkt_data[0] = uint32_t(type) | (uint32_t(request) << 8) | (uint32_t(value) << 16);
			pkt_data[1] = uint32_t(index) | (uint64_t(length) << 16);
		}
        
        __INLINE setup_pkt_t()
		{
		}
		union __attribute__((aligned(4), packed)) {
			struct {
				uint8_t bmRequestType;
				uint8_t bRequest;
				uint16_t wValue;
				uint16_t wIndex;
				uint16_t wLength;
			};
			uint32_t pkt_data[2];
            uint16_t pkt_data16[4];
		};
        
        /* STM32的USB buffer相当的奇怪，不知道能否用下面的结构体来描述
        union __attribute__((aligned(4), packed)) {
			struct {
				uint8_t bmRequestType;
				uint8_t bRequest;
                uint16_t  unused_pack1;
				uint16_t wValue;
                uint16_t  unused_pack2;
				uint16_t wIndex;
                uint16_t  unused_pack3;
				uint16_t wLength;
                uint16_t  unused_pack3;
			};
		    // uint32_t pkt_data[3]; //STM32 不支持32位访问
            uint16_t pkt_data16[8];
		};
        */
	};
	namespace hal {
        enum{
            BULK = 0,
            CONTROL,
            ISO,
            INTERRUPT,
        };
		namespace transaction {
			class ep_impl_t {
			public:
				enum {
					EP_OUT = 1,
					EP_IN = 2,
				};
				static const uint8_t _ep_pktsz = 64;
				__INLINE ep_impl_t() { }
				__INLINE void config(uint_fast8_t type, uint_fast8_t num, bool dir)
				{
                    (&USB_FS.USB_EP0R)[num]().EP_TYPE(type).CTR_RX(1).CTR_TX(1);
                    this->dir = dir;
                    if(dir){
                        _SetEPTxStatus(num, EP_TX_STALL);
                    }else{
                        _SetEPRxStatus(num, EP_RX_VALID);
                        _SetEPRxCount(num, _ep_pktsz);
                    }
                    _ClearEP_KIND(num);
                    _SetEPAddress(num, num);
                    ///xy STM32中一个ep_cfg可以同时控制in和out。不用设置direction
                    ///xy 已经通过上面的宏设置好了cfg,这里传入dir，通过dir来获取数据的addr
                    config(num, 0);
				}
                __WEAK void config(uint_fast8_t num, uint_fast32_t cfg)
				{
                    ///xy 通过ep num得到相关ep寄存，由于STM32可以由在同一个ep上配置成in和out
                    ///xy 这里可能需要换一种计算偏移的方式
                    decltype(reg) p = num;
                    reg = p;
                    uint32_t addr_offset;

                    if(dir){
                        ///xy IN端点，获取tx buffer的位置
                        addr_offset = num*2*_ep_pktsz + GET_TOTOAL_EP_COUNT()*8;
                        _SetEPTxAddr(num, addr_offset);
                    }else{
                        ///xy OUT端点，获取rx buffer的位置
                        addr_offset = (num*2 + 1)*_ep_pktsz + GET_TOTOAL_EP_COUNT()*8;
                        _SetEPRxAddr(num, addr_offset);
                    }
                    ///xy 得到 addr的物理地址
                    if(dir){
                        addr_in = (uint32_t*)(PMAAddr + addr_offset*2);
                    }else{
                        addr_out = (uint32_t*)(PMAAddr + addr_offset*2);
                    }
				}
				__WEAK void set_stall()
				{
                    if(dir){
                        _SetEPRxStatus(reg, EP_TX_STALL);
                    }else{
                        _SetEPRxStatus(reg, EP_RX_STALL);
                    }
				}
				__WEAK void clear_stall()
				{
                    if(dir){
                        _SetEPRxStatus(reg, EP_TX_NAK);
                    }else{
                        _SetEPRxStatus(reg, EP_RX_VALID);
                    }
                    _ClearEP_KIND(reg);
				}
				__WEAK bool in(const void* buffer, uint_fast16_t length, uint_fast16_t max_length)
				{
					inbuf = reinterpret_cast<const uint8_t*>(buffer);
					bool r = false;
					uint_fast16_t len = max_length;
                    
					if (length < max_length) {
						len = length;
						if (length % _ep_pktsz == 0)
							r = true;
					}
					zlp = r;
					count = len;
					return in();
				}
				__WEAK bool in()
				{
                    //dbg()<<"in:"<<endl;
					const uint8_t* buf = nullptr;
					int_fast16_t length = count;
					if ((length << 1) + zlp > 0) {
						uint_fast16_t len = _ep_pktsz;
						count = length - len;
						if (count < 0)
							len = length;
						if (len != 0) {
							buf = inbuf;
							inbuf += len;
							length = len;
                            volatile uint16_t* dst = (volatile uint16_t*)(addr_in);
                            auto* src = reinterpret_cast<const uint16_t*>(buf);
							do {
								*dst++ = *src++;
                                dst++;
								__asm__ __volatile__("" : "+r" (dst), "+r" (src));
                            } while ((length -= sizeof(uint16_t)) > 0);
						}

                        _SetEPTxCount( reg, len);
                        _SetEPTxStatus(reg, EP_TX_VALID);
						return false;
					}
					return true;
				}
				__WEAK void out(void* buffer, uint_fast16_t length)
				{
					outbuf = reinterpret_cast<uint8_t*>(buffer);
					count = length;
					num = 0;
					if (length > _ep_pktsz)
						length = _ep_pktsz;
                    _SetEPRxCount( reg, length);
				}
				__WEAK uint_fast16_t out()
				{
					uint_fast16_t len = count;
                    uint_fast8_t n = _GetEPRxCount(reg);
					if (len > n)
						len = n;
                    auto dst = reinterpret_cast<volatile uint16_t*>(outbuf);
                    auto* src = reinterpret_cast<volatile uint16_t*>(addr_out);
					n = len;
                    while (n >= sizeof(uint16_t)) {
                        n -= sizeof(uint16_t);
						*dst++ = *src++;
                        src++;
						__asm__ __volatile__("" : "+r" (dst), "+r" (src), "+r" (n));
					}
					if (n != 0) {
                        volatile uint16_t* p = reinterpret_cast<volatile uint16_t*>(dst);
                        uint16_t data = *src;
						do {
							*p++ = data;
                            data >>= 16;
						} while (--n);
						__asm__ __volatile__("" :: "r" (data));
					}
					register uint_fast16_t r = num + len;
					num = r;
					n = count - len;
					if (n != 0 && len == _ep_pktsz) {
						count = n;
						outbuf += _ep_pktsz;
						if (n > _ep_pktsz)
							n = _ep_pktsz;
                        _SetEPRxCount( reg, n);
						r = 0;
					}
					return r;
				}

			protected:
				union {
					const uint8_t* inbuf;
					uint8_t* outbuf;
				};
				int16_t count;
				union {
					bool zlp;
					uint16_t num;
				};

                uint32_t reg;
                bool  dir;
				volatile uint32_t* addr_in;   // address for in endpoint
                volatile uint32_t* addr_out;  // address for out endpoint
			};
		}
		namespace transfer {
			namespace control {
				class ep_impl_t : public transaction::ep_impl_t {
				public:
					__INLINE ep_impl_t() { }
					__WEAK void out(void* buffer, uint_fast16_t length)
					{
                        if(length != 0){
                            _SetEPRxStatus(reg, EP_RX_VALID);
                            transaction::ep_impl_t::out(buffer, length);
                        }else{
                            transaction::ep_impl_t::in(buffer, length, _ep_pktsz);
                        }
						
					}
					__WEAK uint_fast16_t out()
					{
						uint_fast16_t n = transaction::ep_impl_t::out();
						if (n != 0) {
                            _ClearDTOG_TX(reg);
                            _SetEPTxStatus(reg, EP_TX_STALL);
                            _SetEPTxCount(reg, 0);
						}
						return n;
					}
					__WEAK bool in(const void* buffer, uint_fast16_t length, uint_fast16_t max_length)
					{
						return transaction::ep_impl_t::in(buffer, length, max_length);
					}
					__WEAK bool in()
					{
						bool r = transaction::ep_impl_t::in();
						if (r) {
                            _SetEPRxStatus(reg, EP_TX_STALL);
                            _SetEPTxCount(reg, 0);
						}
						return r;
					}
				};
			}
			namespace isochronous {
				using transaction::ep_impl_t;
			}
			namespace bulk {
				using transaction::ep_impl_t;
			}
			namespace interrupt {
				using transaction::ep_impl_t;
			}
		}
		template<typename USB, typename EP0>
		class usbd_impl_t : public EP0 {
		protected:
			__INLINE constexpr usbd_impl_t() { }

		public:
			__INLINE void enable_irq()
			{
                uint32_t tmppriority = 0x00, tmppre = 0x00, tmpsub = 0x0F;
                tmppriority = (0x700 - ((SCB->AIRCR) & (uint32_t)0x700))>> 0x08;
                tmppre = (0x4 - tmppriority);
                tmpsub = tmpsub >> tmppriority;

                tmppriority = (uint32_t)2 << tmppre;
                tmppriority |=  0 & tmpsub;
                tmppriority = tmppriority << 0x04;
        
                NVIC->IP[USB_LP_CAN_RX0_IRQn] = tmppriority;
    
                /* Enable the Selected IRQ Channels --------------------------------------*/
                NVIC->ISER[USB_LP_CAN_RX0_IRQn >> 0x05] =
                (uint32_t)0x01 << (USB_LP_CAN_RX0_IRQn & (uint8_t)0x1F);
                _SetCNTR(IMR_MSK);
			}

			__INLINE void disable_irq()
			{
                NVIC->ICER[USB_LP_CAN_RX0_IRQn >> 0x05] =
                (uint32_t)0x01 << (USB_LP_CAN_RX0_IRQn & (uint8_t)0x1F);
                _SetCNTR(0);
			}

			__INLINE void open(bool intr)
			{
                RCC.AHBENR().IOPAEN(1);
                GPIOA.MODER().MODER11(GPIO_Mode_AF).MODER12(GPIO_Mode_AF);
                GPIOA.OTYPER().OT11(GPIO_OType_PP).OT12(GPIO_OType_PP);
                GPIOA.OSPEEDR().OSPEEDR11(GPIO_Speed_50MHz).OSPEEDR12(GPIO_Speed_50MHz);
                GPIOA.PUPDR().PUPDR11(GPIO_PuPd_NOPULL).PUPDR12(GPIO_PuPd_NOPULL);
                GPIOA.AFRH().AFRH11(GPIO_AF_14).AFRH12(GPIO_AF_14);
                
				if (intr) {
					///xy *reinterpret_cast<volatile uint32_t*>(0xe000e100) = 1 << USBD_IRQn;
					enable_irq();
				}
                if(GET_SYS_CLOCK() == 72000000){
                    RCC.CFGR().USBPRES(0);  // 1div5
                }else if(GET_SYS_CLOCK() == 48000000){
                    RCC.CFGR().USBPRES(1);  //  div1
                }else{
                    ///xy program must no reach here
                }
                RCC.APB1ENR().USBEN(1);

                _SetCNTR(CNTR_FRES);
                
                _SetCNTR(0);
                _SetISTR(0);
                _SetCNTR(CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM);
                _SetISTR(0);                
                _SetCNTR(IMR_MSK);
                _SetBTABLE(BTABLE_ADDRESS);
                _SetDADDR(0 | DADDR_EF);
			}

			__INLINE void close()
			{
				disable_irq();
                
                _SetCNTR(CNTR_FRES);
                _SetISTR(0);
                _SetCNTR(CNTR_FRES + CNTR_PDWN);
			}

			__INLINE void isr()
			{
                do{
                    auto& usb = *static_cast<USB*>(this);
                    auto wIstr = _GetISTR();
                    if (wIstr & ISTR_RESET){
                        _SetISTR((uint16_t)CLR_RESET);
                        usb.attach();
                        if (usb.reset()){
                            usb.attach();
                            break;
                        }else{
                        }
                    }
                    if (wIstr & ISTR_WKUP){
                        _SetISTR((uint16_t)CLR_WKUP);
                        if (usb.resume()){
                            usb.attach();
                            
                            uint16_t wCNTR;
                            wCNTR = _GetCNTR();
                            wCNTR &= (~CNTR_LPMODE);
                            _SetCNTR(wCNTR);
                            _SetCNTR(IMR_MSK);
                        }
                    }
                    if( wIstr & ISTR_SUSP){
                        usb.suspend();
                        {
                            uint16_t wCNTR;
                            wCNTR = _GetCNTR();
                            wCNTR |= CNTR_FSUSP;
                            _SetCNTR(wCNTR);
                            
                            wCNTR = _GetCNTR();
                            wCNTR |= CNTR_LPMODE;
                            _SetCNTR(wCNTR);
                        }
                        _SetISTR((uint16_t)CLR_SUSP);
                    }
                    while (((wIstr = _GetISTR()) & ISTR_CTR) != 0)  {
                        uint8_t	EPindex = (uint8_t)(wIstr & ISTR_EP_ID);
                        uint16_t wEPVal;
                        istr = wIstr;
                        if (EPindex == 0) {
                            wEPVal = _GetENDPOINT(ENDP0);
                            if ((wIstr & ISTR_DIR) == 0){ 
                                _ClearEP_CTR_TX(ENDP0);
                                usb.ep_event(0);
                            } else {
                                _ClearEP_CTR_RX(ENDP0);
                                if ((wEPVal &EP_SETUP) != 0){
                                    setup_pkt_t* pkt = &PKT;
                                    uint16_t* buffer = (uint16_t*)(PMAAddr + (uint8_t *)(_GetEPRxAddr(ENDP0) * 2));
                                    pkt->pkt_data16[0] = *buffer++;buffer++;
                                    pkt->pkt_data16[1] = *buffer++;buffer++;
                                    pkt->pkt_data16[2] = *buffer++;buffer++;
                                    pkt->pkt_data16[3] = *buffer;
                                    usb.setup(pkt->bmRequestType, pkt->bRequest, pkt->wValue, pkt->wIndex, pkt->wLength);
                                    _SetEPRxCount(ENDP0,64);
                                    _SetEPRxStatus(ENDP0,EP_RX_VALID);
                                }else if ((wEPVal & EP_CTR_RX) != 0){
                                    usb.ep_event(0);
                                    _SetEPRxCount(ENDP0,64);
                                    _SetEPRxStatus(ENDP0,EP_RX_VALID);
                                }
                            }
                        } else { 
                            wEPVal = _GetENDPOINT(EPindex);
                            if ((wEPVal & EP_CTR_RX) != 0) {
                                _ClearEP_CTR_RX(EPindex);
                                usb.ep_event(EPindex);
                            }
                            if ((wEPVal & EP_CTR_TX) != 0) {
                                _ClearEP_CTR_TX(EPindex);
                                usb.ep_event(EPindex);
                            }
                        }
                    }
                    return;
                }while(false);
                _SetBTABLE(BTABLE_ADDRESS);
                
                EP0::config(CONTROL, 0, true);
                EP0::config(CONTROL, 0 , false);
                _SetDADDR(0 | DADDR_EF);
			}
			__PURE bool attach();
			__PURE bool detach();
			__PURE bool reset();
			__PURE bool suspend();
			__PURE bool resume();

		protected:
			__INLINE void event()
			{
				/// using namespace sfr::usbd;
				auto& usb = *static_cast<USB*>(this);
                setup_pkt_t* pkt = &PKT;
				__asm__ __volatile__("" : "+r" (pkt));
                
                if((istr & ISTR_DIR) == 0) {
                    // Todo: we need delay a while.
                    volatile int i = 200; while(i--);
					usb.transact_in(pkt->bmRequestType, pkt->bRequest, pkt->wValue, pkt->wIndex, pkt->wLength);
				} else {
					usb.transact_out(pkt->bmRequestType, pkt->bRequest, pkt->wValue, pkt->wIndex, pkt->wLength);
				}
			}

			__INLINE bool set_address(uint_fast8_t address)
			{
                _SetDADDR(address | DADDR_EF);
				return true;
			}

            setup_pkt_t PKT;
            uint16_t istr;
		};
	}
}

#endif	// __USB_NUMICRO_H
