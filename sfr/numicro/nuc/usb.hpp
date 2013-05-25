#ifndef __SFR_USB_HPP
#define __SFR_USB_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace usbd {                                // USB Control Registers
        enum {
            USBD_IRQn = 23
        };

        enum {
            USBD_ADDR = APB1_BASE + 0x60000
        };

        union inten_t {                             // USB Interrupt Enable Register (USB_INTEN)
            __SFR(inten_t, uint32_t, 0)
            sfb_t<inten_t, 0, 1> BUS_IE;
            sfb_t<inten_t, 1, 1> USB_IE;
            sfb_t<inten_t, 2, 1> FLDET_IE;
            sfb_t<inten_t, 3, 1> WAKEUP_IE;
            sfb_t<inten_t, 8, 1> WAKEUP_EN;
            sfb_t<inten_t, 15, 1> INNAK_EN;
        };

        union intsts_t {                            // USB Interrupt Event Status Register (USB_INTSTS)
            __SFR(intsts_t, uint32_t, 0)
            sfb_t<intsts_t, 0, 1> BUS_STS;
            sfb_t<intsts_t, 1, 1> USB_STS;
            sfb_t<intsts_t, 2, 1> FLDET_STS;
            sfb_t<intsts_t, 3, 1> WAKEUP_STS;
            sfb_t<intsts_t, 16, 6> EPEVT;
            sfb_t<intsts_t, 16, 1> EPEVT0;
            sfb_t<intsts_t, 17, 1> EPEVT1;
            sfb_t<intsts_t, 18, 1> EPEVT2;
            sfb_t<intsts_t, 19, 1> EPEVT3;
            sfb_t<intsts_t, 20, 1> EPEVT4;
            sfb_t<intsts_t, 21, 1> EPEVT5;
            sfb_t<intsts_t, 31, 1> SETUP;
        };

        union faddr_t {                             // USB Device Function Address Register (USB_FADDR)
            __SFR(faddr_t, uint32_t, 0)
            sfb_t<faddr_t, 0, 7> FADDR;
        };

        union epsts_t {                             // USB Endpoint Status Register (USB_EPSTS)
            __SFR(epsts_t, uint32_t, 0)
            sfb_t<epsts_t, 7, 1> OVERRUN;
            sfb_t<epsts_t, 8, 18> EPSTS;
            sfb_t<epsts_t, 8, 3> EPSTS0;
            sfb_t<epsts_t, 11, 3> EPSTS1;
            sfb_t<epsts_t, 14, 3> EPSTS2;
            sfb_t<epsts_t, 17, 3> EPSTS3;
            sfb_t<epsts_t, 20, 3> EPSTS4;
            sfb_t<epsts_t, 23, 3> EPSTS5;
        };

        union attr_t {                              // USB Bus Status and Attribution Register (USB_ATTR)
            __SFR(attr_t, uint32_t, 0)
            sfb_t<attr_t, 0, 1> USBRST;
            sfb_t<attr_t, 1, 1> SUSPEND;
            sfb_t<attr_t, 2, 1> RESUME;
            sfb_t<attr_t, 3, 1> TIMEOUT;
            sfb_t<attr_t, 4, 1> PHY_EN;
            sfb_t<attr_t, 5, 1> RWAKEUP;
            sfb_t<attr_t, 7, 1> USB_EN;
            sfb_t<attr_t, 8, 1> DPPU_EN;
            sfb_t<attr_t, 9, 1> PWRDN;
            sfb_t<attr_t, 10, 1> BYTEM;
        };

        union fldet_t {                             // Floating detection Register (USB_FLDET)
            __SFR(fldet_t, uint32_t, 0)
            sfb_t<fldet_t, 0, 1> FLDET;
        };

        union bufseg_t {                            // Buffer Segmentation Register (USB_BUFSEG)
            __SFR(bufseg_t, uint32_t, 0)
            sfb_t<bufseg_t, 3, 6> BUFSEG;
        };

        union mxpld_t {                             // Maximal Payload Register (USB_MXPLD)
            __SFR(mxpld_t, uint32_t, 0)
            sfb_t<mxpld_t, 0, 9> MXPLD;
        };

        union cfg_t {                               // Configuration Register (USB_CFG)
            __SFR(cfg_t, uint32_t, 0)
            sfb_t<cfg_t, 0, 4> EP_NUM;
            sfb_t<cfg_t, 4, 1> ISOCH;
            sfb_t<cfg_t, 5, 2> STATE;
            sfb_t<cfg_t, 7, 1> DSQ_SYNC;
            sfb_t<cfg_t, 9, 1> CSTALL;
        };

        union cfgp_t {                              // Extra Configuration Register (USB_CFGP)
            __SFR(cfgp_t, uint32_t, 0)
            sfb_t<cfgp_t, 0, 1> CLRRDY;
            sfb_t<cfgp_t, 1, 1> SSTALL;
        };

        union drvse0_t {                            // USB Drive SE0 Register (USB_DRVSE0)
            __SFR(drvse0_t, uint32_t, 0)
            sfb_t<drvse0_t, 0, 1> DRVSE0;
        };

        struct usbd_t {
            volatile sfr_t<inten_t> INTEN;          // USB Interrupt Enable Register (USB_INTEN)
            volatile sfr_t<intsts_t> INTSTS;        // USB Interrupt Event Status Register (USB_INTSTS)
            volatile sfr_t<faddr_t> FADDR;          // USB Device Function Address Register (USB_FADDR)
            const volatile sfr_t<epsts_t> EPSTS;    // USB Endpoint Status Register (USB_EPSTS)
            volatile sfr_t<attr_t> ATTR;            // USB Bus Status and Attribution Register (USB_ATTR)
            const volatile sfr_t<fldet_t> FLDET;    // Floating detection Register (USB_FLDET)
            volatile sfr_t<bufseg_t> BUFSEG;        // Buffer Segmentation Register (USB_BUFSEG)
            uint32_t :32;
            struct {
                volatile sfr_t<bufseg_t> BUFSEG;    // Buffer Segmentation Register (BUFSEG)
                volatile sfr_t<mxpld_t> MXPLD;      // Maximal Payload Register (USB_MXPLD)
                volatile sfr_t<cfg_t> CFG;          // Configuration Register (USB_CFG)
                volatile sfr_t<cfgp_t> CFGP;        // Extra Configuration Register (USB_CFGP)
            } EP[6];
            uint64_t :64;
            uint64_t :64;
            volatile sfr_t<drvse0_t> DRVSE0;        // USB Drive SE0 Register (USB_DRVSE0)
            uint64_t :32;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
            union {
                volatile uint8_t SRAM8[512];
                volatile uint32_t SRAM32[128];
            };
        };

        extern usbd_t USBD;     SFR_ADDR(USBD, USBD_ADDR);
    }
}

#endif  // __SFR_USB_HPP
