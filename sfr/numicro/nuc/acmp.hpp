#ifndef __SFR_ACMP_HPP
#define __SFR_ACMP_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace acmp {                                // Analog Comparator Control Registers
        enum {
            ACMP_IRQn = 25
        };

        enum {
            ACMP_ADDR = APB1_BASE + 0xD0000
        };

        union cmpcr_t {                             // CMP Control Register (CMPCR)
            __SFR(cmpcr_t, uint32_t, 0)
            sfb_t<cmpcr_t, 0, 1> CMPEN;
            sfb_t<cmpcr_t, 1, 1> CMPIE;
            sfb_t<cmpcr_t, 2, 1> CMP_HYSEN;
            sfb_t<cmpcr_t, 4, 1> CN;
        };

        union cmpsr_t {                             // CMP Status Register (CMPSR)
            __SFR(cmpsr_t, uint32_t, 0)
            sfb_t<cmpsr_t, 0, 1> CMPF0;
            sfb_t<cmpsr_t, 0, 1> CMPF1;
            sfb_t<cmpsr_t, 0, 1> CO0;
            sfb_t<cmpsr_t, 0, 1> CO1;
        };

        struct acmp_t {
            union {
                volatile sfr_t<cmpcr_t> CMPCR[2];   // CMP Control Register (CMPCR)
                struct {
                    volatile sfr_t<cmpcr_t> CMPCR0; // CMP0 Control Register (CMP0CR)
                    volatile sfr_t<cmpcr_t> CMPCR1; // CMP1 Control Register (CMP1CR)
                };
            };
            volatile sfr_t<cmpsr_t> CMPSR;          // CMP Status Register (CMPSR)
        };

        extern acmp_t ACMP;     SFR_ADDR(ACMP, ACMP_ADDR);
    }
}

#endif  // __SFR_ACMP_HPP
