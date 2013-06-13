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
                    ///xy using namespace sfr::usbd;
                    ///xy sfr::sfr_t<sfr::usbd::cfg_t> cfg;
//					cfg(0).EP_NUM(num, false).ISOCH(type == 1, false).STATE(dir + 1, false).CSTALL(1);
                    ///xy cfg(0).EP_NUM(num, false).ISOCH(type == 1, false).STATE(dir + 1, false);
                    ///xy config(num, cfg);
                    ///xy 根据type,num,dir设置ep的类型
                    ///xy Nuc120中使用的是 USB_CFGx来设置ep的类型和方向 (NUC100s_TRM 5.4.6)
                    ///xy 而STM32中使用的是 USB_EPnR来设置ep的类型和方向 (STM32_RM0008 23.5.2)
                    ///xy valid types in STM32 are EP_BULK, EP_CONTROL, EP_ISOCHRONOUS, EP_INTERRUPT
                    //dbg()<<"Config ep"<<num<<" dir:"<<dir<<" type:"<<type<<endl;
                    (&USB_FS.USB_EP0R)[num]().EP_TYPE(type).CTR_RX(1).CTR_TX(1);
                    //if(num == 0){
                    //    dbg()<<"Set ep as control"<<endl;
                    //    USB_FS.USB_EP0R().EP_TYPE.(CONTROL).CTR_RX(1).CTR_TX(1);
                    //    //_SetEPType(num, EP_CONTROL);
                    //}else{
                    //    dbg()<<"Set ep as int"<<endl;
                    //    
                        //_SetEPType(num, EP_INTERRUPT);
                    //}
                    this->dir = dir;
                    if(dir){
                        ///xy dir = 1, M0中STATE为2，表示IN
                        _SetEPTxStatus(num, EP_TX_STALL);
                        _SetEPTxStatus(num, EP_TX_STALL);
                    }else{
                        ///xy dir = 0, M0中STATE为1，表示OUT
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
                    ///xy using namespace sfr::usbd;
                    ///xy 通过ep num得到相关ep寄存，由于STM32可以由在同一个ep上配置成in和out
                    ///xy 这里可能需要换一种计算偏移的方式
                    ///xy decltype(reg) p = &USBD.EP[num];
                    ///xy CFG已经写入
                    ///xy p->CFG(cfg);
                    decltype(reg) p = num;
                    reg = p;
                    uint32_t addr_offset;
                    
                    //dbg()<<"Config ep"<<num<<endl;
                    ///xy 根据num和in还是out得到ep的buffer地址
                    ///xy STM32的BTABLE除了存放数据
                    ///xy 还要存放ADDRn_TX, COUNTn_TX, ADDRn_RX, COUNTn_RX
                    ///xy addr需要从这些信息之后开始计算
                    ///xy addr = &USBD.SRAM32[num * 16];
                    if(dir){
                        ///xy IN端点，获取tx buffer的位置
                        addr_offset = num*2*_ep_pktsz + GET_TOTOAL_EP_COUNT()*8;
                        _SetEPTxAddr(num, addr_offset);
                        //dbg()<<"Set ep"<<num<<" tx addr:"<<addr_offset<<endl;
                        /// static_assert(addr_offset*1 == 0)
                    }else{
                        ///xy OUT端点，获取rx buffer的位置
                        addr_offset = (num*2 + 1)*_ep_pktsz + GET_TOTOAL_EP_COUNT()*8;
                        _SetEPRxAddr(num, addr_offset);
                        //dbg()<<"Set ep"<<num<<" rx addr:"<<addr_offset<<endl;
                        /// static_assert(addr_offset*1 == 0)
                    }
                    ///xy 得到 addr的物理地址
                    if(dir){
                        addr_in = (uint32_t*)(PMAAddr + addr_offset*2);
                        //dbg()<<"addr_in: "<<HEX(addr_in)<<endl;
                    }else{
                        addr_out = (uint32_t*)(PMAAddr + addr_offset*2);
                        //dbg()<<"addr_out: "<<HEX(addr_out)<<endl;
                    }
                    ///xy 设置buffer对应的
                    ///xy p->BUFSEG(num * _ep_pktsz);

                    ///xy p->CFGP(0).CLRRDY(1);
				}
				__WEAK void set_stall()
				{
                    //using namespace sfr::usbd;
                    //reg->CFGP(0).CLRRDY(1).SSTALL(1);
                    //dbg()<<"set_stall:"<<reg<<" dir:"<<dir<<endl;
                    if(dir){
                        _SetEPRxStatus(reg, EP_TX_STALL);
                    }else{
                        _SetEPRxStatus(reg, EP_RX_STALL);
                    }
				}
				__WEAK void clear_stall()
				{
                    //using namespace sfr::usbd;
                    //reg->CFGP(0).CLRRDY(1);
                    //reg->CFG().DSQ_SYNC(0);
                    //dbg()<<"clear_stall:"<<reg<<" dir:"<<dir<<endl;
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
                    
                    //dbg()<<"in:"<<reg<<" dir:"<<dir<<" len:"<<length<<" max len:"<<max_length<<endl;
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
                            //volatile uint16_t* dst = (volatile uint16_t*)addr;
                            //volatile uint16_t* dst = (uint16_t*)(PMAAddr + (uint8_t *)(_GetEPTxAddr(reg) * 2));
                            volatile uint16_t* dst = (volatile uint16_t*)(addr_in);
                            //dbg()<<"dst: "<<HEX(dst)<<endl;
                            //dbg()<<"addr: "<<HEX(ddt)<<endl;
                            auto* src = reinterpret_cast<const uint16_t*>(buf);
							do {
								*dst++ = *src++;
                                dst++;
								__asm__ __volatile__("" : "+r" (dst), "+r" (src));
                            } while ((length -= sizeof(uint16_t)) > 0);
						}

                        ///xy reg->MXPLD(0).MXPLD(len, false);
                        //dbg()<<"ep"<<reg<<" tx len = "<<len<<endl;
                        _SetEPTxCount( reg, len);
                        _SetEPTxStatus(reg, EP_TX_VALID);
						return false;
					}
					return true;
				}
				__WEAK void out(void* buffer, uint_fast16_t length)
				{
                    //dbg()<<"out:"<<reg<<" dir:"<<dir<<" length:"<<length<<endl;
					outbuf = reinterpret_cast<uint8_t*>(buffer);
					count = length;
					num = 0;
					if (length > _ep_pktsz)
						length = _ep_pktsz;

                    ///xy reg->MXPLD(0).MXPLD(length, false);
                    _SetEPRxCount( reg, length);
				}
				__WEAK uint_fast16_t out()
				{
					uint_fast16_t len = count;
                    uint_fast8_t n = _GetEPRxCount(reg);///xy reg->MXPLD;
                    //dbg()<<"out"<<endl;
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
                        //reg->MXPLD(0).MXPLD(n, false);
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

                ///xy 这里使用offset来表示eg reg的位置，使用时需要加上EP0REG的偏移
                ///xy decltype(&sfr::usbd::USBD.EP[0]) reg;
                uint32_t reg;
                bool  dir;
				volatile uint32_t* addr_in;
                volatile uint32_t* addr_out;
			};
		}
		namespace transfer {
			namespace control {
				class ep_impl_t : public transaction::ep_impl_t {
				public:
					__INLINE ep_impl_t() { }
					__WEAK void out(void* buffer, uint_fast16_t length)
					{
//						reg->CFG().STATE(length != 0 ? EP_OUT : EP_IN, false).DSQ_SYNC(1).CSTALL(1);
                        ///xy reg->CFG().STATE(length != 0 ? EP_OUT : EP_IN, false).DSQ_SYNC(1);
                        //dbg()<<"control out:"<<reg<<" dir:"<<dir<<" length:"<<length<<endl;
                        if(length != 0){
                            _SetEPRxStatus(reg, EP_RX_VALID);
                            transaction::ep_impl_t::out(buffer, length);
                            //_ClearDTOG_RX(reg);
                        }else{
                            ///后面的out会TX数据
                            //_SetEPRxStatus(reg, EP_TX_VALID);
                            //_ClearDTOG_TX(reg);
                            transaction::ep_impl_t::in(buffer, length, _ep_pktsz);
                        }
						
					}
					__WEAK uint_fast16_t out()
					{
						uint_fast16_t n = transaction::ep_impl_t::out();
                        //dbg()<<"control out"<<endl;
						if (n != 0) {
//							reg->CFG().STATE(EP_IN).DSQ_SYNC(1).CSTALL(1);
                            ///xy reg->CFG().STATE(EP_IN).DSQ_SYNC(1);
                            ///xy reg->MXPLD(0).MXPLD(0);
                            _ClearDTOG_TX(reg);
                            _SetEPTxStatus(reg, EP_TX_STALL);
                            _SetEPTxCount(reg, 0);
						}
						return n;
					}
					__WEAK bool in(const void* buffer, uint_fast16_t length, uint_fast16_t max_length)
					{
//						reg->CFG().STATE(EP_IN).DSQ_SYNC(1).CSTALL(1);
                        ///xy reg->CFG().STATE(EP_IN).DSQ_SYNC(1);
                        //dbg()<<"control in:"<<reg<<" dir:"<<dir<<" len:"<<length<<" max_len:"<<max_length<<endl;
                        //_ClearDTOG_RX(reg);
						return transaction::ep_impl_t::in(buffer, length, max_length);
					}
					__WEAK bool in()
					{
						bool r = transaction::ep_impl_t::in();
                        //dbg()<<"control in "<<r<<endl;
						if (r) {
//							reg->CFG().STATE(EP_OUT).DSQ_SYNC(1);
                            ///xy reg->CFG().STATE(EP_OUT).DSQ_SYNC(1).CSTALL(1);
                            ///xy reg->MXPLD(0).MXPLD(0);
                            _SetEPRxStatus(reg, EP_TX_STALL);
                            //_ClearDTOG_TX(reg);
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
                ///xy using namespace sfr::usbd;
                ///xy USBD.INTEN(0).BUS_IE(1).USB_IE(1).FLDET_IE(1).WAKEUP_IE(1).WAKEUP_EN(1);
                ///
                //NVIC_InitTypeDef NVIC_InitStructure;
                //NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
                //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
                //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                //NVIC_Init(&NVIC_InitStructure);
                //dbg()<<"enable_irq"<<endl;
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
                
                /// enable USB interrupts
                _SetCNTR(IMR_MSK);
			}

			__INLINE void disable_irq()
			{
                //dbg()<<"disable_irq"<<endl;
                ///xy using namespace sfr::usbd;
                ///xy USBD.INTEN(0);
                //NVIC_InitTypeDef NVIC_InitStructure;
                //NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
                //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
                //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                //NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
                //NVIC_Init(&NVIC_InitStructure);
                
                //uint32_t tmppriority = 0x00, tmppre = 0x00, tmpsub = 0x0F;
                NVIC->ICER[USB_LP_CAN_RX0_IRQn >> 0x05] =
                (uint32_t)0x01 << (USB_LP_CAN_RX0_IRQn & (uint8_t)0x1F);
                //sfr::gpio::GPIOA;
                //(&NVIC.ICER0)[USB_LP_CAN_RX0_IRQn >> 0x05] = 1<< (USB_LP_CAN_RX0_IRQn & (uint8_t)0x1F);
                
                /// disable all USB interrupts
                _SetCNTR(0);
			}

			__INLINE void open(bool intr)
			{
				///xy using namespace sfr::usbd;
                //dbg()<<"open "<<intr<<endl;
                // We need config the GPIO for the STM32F3xx
                //setup_io_usb();
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
                ///xy BD.ATTR(0x40).PHY_EN(1).USB_EN(1).DPPU_EN(1).PWRDN(1).BYTEM(0);			// Enable USB and enable PHY
                ///xy USBD.DRVSE0(0).DRVSE0(0);
//#define RCC_OFFSET                (RCC_BASE - PERIPH_BASE)
//#define USBPRE_BitNumber          0x16
//#define CFGR_OFFSET               (RCC_OFFSET + 0x04)
//#define CFGR_USBPRE_BB            (PERIPH_BB_BASE + (CFGR_OFFSET * 32) + (USBPRE_BitNumber * 4))
                if(GET_SYS_CLOCK() == 72000000){
                    //dbg()<<"SYS 72MHz"<<endl;
                    RCC.CFGR().USBPRES(0);
                    //*(__IO uint32_t *) CFGR_USBPRE_BB = RCC_USBCLKSource_PLLCLK_1Div5;
                }else if(GET_SYS_CLOCK() == 48000000){
                    //dbg()<<"SYS 48MHz"<<endl;
                    RCC.CFGR().USBPRES(1);
                    //*(__IO uint32_t *) CFGR_USBPRE_BB = RCC_USBCLKSource_PLLCLK_Div1;
                }else{
                    //dbg()<<"SYS not support xMHz"<<endl;
                    ///xy program must no reach here
                }
                RCC.APB1ENR().USBEN(1);
                
                //RCC->APB1ENR |= RCC_APB1Periph_USB;
                
                
                /// general usb power on 
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
				///using namespace sfr::usbd;
                //dbg()<<"close"<<endl;
				disable_irq();
				///USBD.ATTR(0x40);
				///USBD.DRVSE0(0).DRVSE0(1);
                
                _SetCNTR(CNTR_FRES);
                _SetISTR(0);
                _SetCNTR(CNTR_FRES + CNTR_PDWN);
			}

			__INLINE void isr()
			{
                //dbg()<<"USB ISR"<<endl;
				/// using namespace sfr::usbd;
#if 0
				do {
					auto& usb = *static_cast<USB*>(this);
					/// const auto intsts = USBD.INTSTS();
                    const auto intsts = _GetISTR();
					if (intsts.FLDET_STS) {
						const auto fldet = USBD.FLDET();
						USBD.INTSTS(0).FLDET_STS(1);
						if (fldet.FLDET) {
							if (usb.attach())
								break;
						} else {
							usb.detach();
							USBD.ATTR(0x40).RWAKEUP(1).DPPU_EN(1).PWRDN(1).BYTEM(0);				// Disable USB
						}
					} else if (intsts.BUS_STS) {
						const auto attr = USBD.ATTR();
						USBD.INTSTS(0).BUS_STS(1);
						if (attr.USBRST) {
							if (usb.reset())
								break;
						} else if (attr.SUSPEND) {
							if (usb.suspend())
								USBD.ATTR(0x40).USB_EN(1).DPPU_EN(1).PWRDN(1).BYTEM(0);				// Enable USB but disable PHY
						} else if (attr.RESUME) {
							if (usb.resume())
								break;
						}
					} else if (intsts.USB_STS) {
						if (intsts.SETUP) {
							setup_pkt_t* pkt = reinterpret_cast<setup_pkt_t*>(const_cast<uint32_t*>(&USBD.SRAM32[126]));
							__asm__ __volatile__("" : "+r" (pkt));
							USBD.INTSTS(0).SETUP(1);
							usb.setup(pkt->bmRequestType, pkt->bRequest, pkt->wValue, pkt->wIndex, pkt->wLength);
						} else {
							if (intsts.EPEVT) {
								USBD.INTSTS(0).EPEVT(intsts.EPEVT);
								if (intsts.EPEVT0)
									usb.ep_event(0);
								if (intsts.EPEVT1)
									usb.ep_event(1);
								if (intsts.EPEVT2)
									usb.ep_event(2);
								if (intsts.EPEVT3)
									usb.ep_event(3);
								if (intsts.EPEVT4)
									usb.ep_event(4);
								if (intsts.EPEVT5)
									usb.ep_event(5);
							}
						}
					}
					return;
				} while (false);
#endif
                do{
                    auto& usb = *static_cast<USB*>(this);
                    auto wIstr = _GetISTR();
                    if (wIstr & ISTR_RESET){
                        _SetISTR((uint16_t)CLR_RESET);
                        //dbg()<<"ISTR_RESET:"<<endl;
                        usb.attach();
                        if (usb.reset()){
                            /// For STM32, there is no status for device attach
                            //dbg()<<"Attach reset:"<<endl;
                            usb.attach();
                            break;
                        }else{
                            //dbg()<<"Reset fail:"<<endl;
                        }
                    }
                    if (wIstr & ISTR_WKUP){
                        _SetISTR((uint16_t)CLR_WKUP);
                        //dbg()<<"ISTR_WKUP:"<<endl;
                        if (usb.resume()){
                            /// For STM32, there is no status for device attach
                            //dbg()<<"Attach resume:"<<endl;
                            usb.attach();
                            //break;
                            
                            uint16_t wCNTR;
                            wCNTR = _GetCNTR();
                            wCNTR &= (~CNTR_LPMODE);
                            _SetCNTR(wCNTR);
                            _SetCNTR(IMR_MSK);
                        }
                    }
                    if( wIstr & ISTR_SUSP){
                        //dbg()<<"ISTR_SUSP:"<<endl;
                        usb.suspend();
                        {
                            /// STM32 suspend code
                            //dbg()<<"Suspend:"<<endl;
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
                        //dbg()<<"Has ep event"<<endl;
                        if (EPindex == 0) {
                            //dbg()<<"ep0: event"<<endl;
                            wEPVal = _GetENDPOINT(ENDP0);
                            //uint16_t SaveRState = wEPVal & EPRX_STAT;
                            //uint16_t SaveTState = wEPVal & EPTX_STAT;
                            if ((wIstr & ISTR_DIR) == 0){ 
                                _ClearEP_CTR_TX(ENDP0);
                                //dbg()<<"ep0 in"<<endl;
                                usb.ep_event(0);
                            } else {
                                _ClearEP_CTR_RX(ENDP0);
                                if ((wEPVal &EP_SETUP) != 0){
                                    //setup_pkt_t PKT;
                                    setup_pkt_t* pkt = &PKT;
                                    uint16_t* buffer = (uint16_t*)(PMAAddr + (uint8_t *)(_GetEPRxAddr(ENDP0) * 2));
                                    pkt->pkt_data16[0] = *buffer++;buffer++;
                                    pkt->pkt_data16[1] = *buffer++;buffer++;
                                    pkt->pkt_data16[2] = *buffer++;buffer++;
                                    pkt->pkt_data16[3] = *buffer;
                                    //dbg()<<"ep0 setup   state:" << HEX(usb.state)<<endl;
                                    //dbg()<<pkt->bmRequestType<<","<<pkt->bRequest<<","<<pkt->wValue<<","<<pkt->wIndex<<","<<pkt->wLength<<endl;
                                    usb.setup(pkt->bmRequestType, pkt->bRequest, pkt->wValue, pkt->wIndex, pkt->wLength);
                                    _SetEPRxCount(ENDP0,64);
                                    _SetEPRxStatus(ENDP0,EP_RX_VALID);
                                }else if ((wEPVal & EP_CTR_RX) != 0){
                                    //dbg()<<"ep0 out"<<endl;
                                    usb.ep_event(0);
                                    _SetEPRxCount(ENDP0,64);
                                    _SetEPRxStatus(ENDP0,EP_RX_VALID);
                                }
                            }
                            //_SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);
                        } else { 
                            wEPVal = _GetENDPOINT(EPindex);
                            //dbg()<<"ep"<<wEPVal<<" : event"<<endl;
                            if ((wEPVal & EP_CTR_RX) != 0)
                              {
                                /* clear int flag */
                                _ClearEP_CTR_RX(EPindex);

                                /* call OUT service function */
                                usb.ep_event(EPindex);

                              } /* if((wEPVal & EP_CTR_RX) */

                              if ((wEPVal & EP_CTR_TX) != 0)
                              {
                                /* clear int flag */
                                _ClearEP_CTR_TX(EPindex);

                                /* call IN service function */
                                usb.ep_event(EPindex);
                              } /* if((wEPVal & EP_CTR_TX) != 0) */
                        }
                    }
                    return;
                }while(false);
                //dbg()<<"Reset the device after while loop"<<endl;
				/// USBD.FADDR(0);		                        // Init the USB device address to 0x0
                
				/// USBD.BUFSEG(504);	                        // Buffer for setup packet
                
                _SetBTABLE(BTABLE_ADDRESS);

				//EP0::config(0, 0, true);
                
                //_SetEPType(ENDP0, EP_CONTROL);
                //_SetEPTxStatus(ENDP0, EP_TX_STALL);
                
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
                //dbg()<<__func__<<endl;
				/// setup_pkt_t* pkt = reinterpret_cast<setup_pkt_t*>(const_cast<uint32_t*>(&USBD.SRAM32[126]));
                setup_pkt_t* pkt = &PKT;
				__asm__ __volatile__("" : "+r" (pkt));
                
                //uint16_t* buffer = (uint16_t*)(PMAAddr + (uint8_t *)(_GetEPRxAddr(ENDP0) * 2));
                //pkt->pkt_data16[0] = *buffer++;buffer++;
                //pkt->pkt_data16[1] = *buffer++;buffer++;
                //pkt->pkt_data16[2] = *buffer++;buffer++;
                //pkt->pkt_data16[3] = *buffer++;
				/// if (this->reg->CFG().STATE == 2) {
                
                /// 这个地方会有问题
                /// 新塘的状态在各自的ep寄存器中，而STM32的ep寄存器同时支持in和out
                /// 需要用别的方式来检测这里是in还是out
                /// STM32官方库是通过 _GetISTR()来得到ep编号和方向的。然后再调用相应的函数来处理
                /// 参见void CTR_LP(void)函数的实现。在usb_int.c文件中
                if((/*_GetISTR()*/ istr & ISTR_DIR) == 0) {
                    //dbg()<<"transact_in"<<endl;
                    //dbg()<<pkt->bmRequestType<<","<<pkt->bRequest<<","<<pkt->wValue<<","<<pkt->wIndex<<","<<pkt->wLength<<endl;
                    volatile int i = 200; while(i--);
					usb.transact_in(pkt->bmRequestType, pkt->bRequest, pkt->wValue, pkt->wIndex, pkt->wLength);
				} else {
                    //dbg()<<"transact_out"<<endl;
                    //dbg()<<pkt->bmRequestType<<","<<pkt->bRequest<<","<<pkt->wValue<<","<<pkt->wIndex<<","<<pkt->wLength<<endl;
					usb.transact_out(pkt->bmRequestType, pkt->bRequest, pkt->wValue, pkt->wIndex, pkt->wLength);
				}
			}

			__INLINE bool set_address(uint_fast8_t address)
			{
				///xy using namespace sfr::usbd;
				///xy USBD.FADDR(address);
                //dbg()<<"Set address:"<<address<<endl;
                _SetDADDR(address | DADDR_EF);
                //USB_FS.DADDR(address|DADDR_EF);
                
				return true;
			}
            // Remember last pkt
            setup_pkt_t PKT;
            uint16_t istr;
		};
	}
}

#endif	// __USB_NUMICRO_H
