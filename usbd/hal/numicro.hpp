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

#ifndef __USBD_HAL_H
#define __USBD_HAL_H

#ifndef NDEBUG
#define NDEBUG
#endif

#include <assert.h>
#include <cstdint>
#include <type_traits>
#include <attribute.h>
#include <sfr/numicro/nuc/usb>

namespace usbd {
	struct __attribute__((aligned(2), packed)) setup_pkt_t {
		__INLINE setup_pkt_t(uint8_t type, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
		{
			pkt_data[0] = uint32_t(type) | (uint32_t(request) << 8) | (uint32_t(value) << 16);
			pkt_data[1] = uint32_t(index) | (uint64_t(length) << 16);
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
		};
	};
	namespace hal {
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
					using namespace sfr::usbd;
					sfr::sfr_t<sfr::usbd::cfg_t> cfg;
//					cfg(0).EP_NUM(num, false).ISOCH(type == 1, false).STATE(dir + 1, false).CSTALL(1);
					cfg(0).EP_NUM(num, false).ISOCH(type == 1, false).STATE(dir + 1, false);
					config(num, cfg);
				}
				__WEAK void config(uint_fast8_t num, uint_fast32_t cfg)
				{
					using namespace sfr::usbd;
					decltype(reg) p = &USBD.EP[num];
					p->CFG(cfg);
					reg = p;
					addr = &USBD.SRAM32[num * 16];
					p->BUFSEG(num * _ep_pktsz);
					p->CFGP(0).CLRRDY(1);
				}
				__WEAK void set_stall()
				{
					using namespace sfr::usbd;
					reg->CFGP(0).CLRRDY(1).SSTALL(1);
				}
				__WEAK void clear_stall()
				{
					using namespace sfr::usbd;
					reg->CFGP(0).CLRRDY(1);
					reg->CFG().DSQ_SYNC(0);
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
							auto dst = addr;
							auto* src = reinterpret_cast<const uint32_t*>(buf);
							do {
								*dst++ = *src++;
								__asm__ __volatile__("" : "+r" (dst), "+r" (src));
							} while ((length -= sizeof(uint32_t)) > 0);
						}
						reg->MXPLD(0).MXPLD(len, false);
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
					reg->MXPLD(0).MXPLD(length, false);
				}
				__WEAK uint_fast16_t out()
				{
					uint_fast16_t len = count;
					uint_fast8_t n = reg->MXPLD;
					if (len > n)
						len = n;
					auto dst = reinterpret_cast<uint32_t*>(outbuf);
					auto* src = addr;
					n = len;
					while (n >= sizeof(uint32_t)) {
						n -= sizeof(uint32_t);
						*dst++ = *src++;
						__asm__ __volatile__("" : "+r" (dst), "+r" (src), "+r" (n));
					}
					if (n != 0) {
						volatile uint8_t* p = reinterpret_cast<volatile uint8_t*>(dst);
						uint32_t data = *src;
						do {
							*p++ = data;
							data >>= 8;
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
						reg->MXPLD(0).MXPLD(n, false);
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
				decltype(&sfr::usbd::USBD.EP[0]) reg;
				volatile uint32_t* addr;
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
						reg->CFG().STATE(length != 0 ? EP_OUT : EP_IN, false).DSQ_SYNC(1);
						transaction::ep_impl_t::out(buffer, length);
					}
					__WEAK uint_fast16_t out()
					{
						uint_fast16_t n = transaction::ep_impl_t::out();
						if (n != 0) {
//							reg->CFG().STATE(EP_IN).DSQ_SYNC(1).CSTALL(1);
							reg->CFG().STATE(EP_IN).DSQ_SYNC(1);
							reg->MXPLD(0).MXPLD(0);
						}
						return n;
					}
					__WEAK bool in(const void* buffer, uint_fast16_t length, uint_fast16_t max_length)
					{
//						reg->CFG().STATE(EP_IN).DSQ_SYNC(1).CSTALL(1);
						reg->CFG().STATE(EP_IN).DSQ_SYNC(1);
						return transaction::ep_impl_t::in(buffer, length, max_length);
					}
					__WEAK bool in()
					{
						bool r = transaction::ep_impl_t::in();
						if (r) {
//							reg->CFG().STATE(EP_OUT).DSQ_SYNC(1);
							reg->CFG().STATE(EP_OUT).DSQ_SYNC(1).CSTALL(1);
							reg->MXPLD(0).MXPLD(0);
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
				using namespace sfr::usbd;
				USBD.INTEN(0).BUS_IE(1).USB_IE(1).FLDET_IE(1).WAKEUP_IE(1).WAKEUP_EN(1);
			}

			__INLINE void disable_irq()
			{
				using namespace sfr::usbd;
				USBD.INTEN(0);
			}

			__INLINE void open(bool intr)
			{
				using namespace sfr::usbd;
				if (intr) {
					*reinterpret_cast<volatile uint32_t*>(0xe000e100) = 1 << USBD_IRQn;
					enable_irq();
				}
				USBD.ATTR(0x40).PHY_EN(1).USB_EN(1).DPPU_EN(1).PWRDN(1).BYTEM(0);			// Enable USB and enable PHY
				USBD.DRVSE0(0).DRVSE0(0);
			}

			__INLINE void close()
			{
				using namespace sfr::usbd;
				disable_irq();
				USBD.ATTR(0x40);
				USBD.DRVSE0(0).DRVSE0(1);
			}

			__INLINE void isr()
			{
				using namespace sfr::usbd;
				do {
					auto& usb = *static_cast<USB*>(this);
					const auto intsts = USBD.INTSTS();
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
				USBD.FADDR(0);											// Init the USB device address to 0x0
				USBD.BUFSEG(504);								 		// Buffer for setup packet
				EP0::config(0, 0, true);
				USBD.ATTR(0x40).PHY_EN(1).USB_EN(1).DPPU_EN(1).PWRDN(1).BYTEM(0);	// Enable USB and enable PHY
			}
			__PURE bool attach();
			__PURE bool detach();
			__PURE bool reset();
			__PURE bool suspend();
			__PURE bool resume();

		protected:
			__INLINE void event()
			{
				using namespace sfr::usbd;
				auto& usb = *static_cast<USB*>(this);
				setup_pkt_t* pkt = reinterpret_cast<setup_pkt_t*>(const_cast<uint32_t*>(&USBD.SRAM32[126]));
				__asm__ __volatile__("" : "+r" (pkt));
				if (this->reg->CFG().STATE == 2) {
					usb.transact_in(pkt->bmRequestType, pkt->bRequest, pkt->wValue, pkt->wIndex, pkt->wLength);
				} else {
					usb.transact_out(pkt->bmRequestType, pkt->bRequest, pkt->wValue, pkt->wIndex, pkt->wLength);
				}
			}

			__INLINE bool set_address(uint_fast8_t address)
			{
				using namespace sfr::usbd;
				USBD.FADDR(address);
				return true;
			}
		};
	}
}

#endif	// __USBD_NUMICRO_H
