#ifndef __SFR_ADC_HPP
#define __SFR_ADC_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace adc {                                 // Analog-Digital-Converter(ADC) Control Registers
        enum {
            ADC_IRQn = 29
        };

        enum {
            ADC_ADDR = APB1_BASE + 0xE0000
        };

        union addr_t {                              // A/D Data Registers (ADDR)
            __SFR(addr_t, uint32_t, 0)
            sfb_t<addr_t, uint32_t, 0, 10> RSLT;
            sfb_t<addr_t, uint32_t, 16, 1> OVERRUN;
            sfb_t<addr_t, uint32_t, 17, 1> VALID;
        };

        union adcr_t {                              // A/D Control Register (ADCR)
            __SFR(adcr_t, uint32_t, 0)
            sfb_t<adcr_t, uint32_t, 0, 1> ADEN;
            sfb_t<adcr_t, uint32_t, 1, 1> ADIE;
            sfb_t<adcr_t, uint32_t, 6, 1> TRGCOND;
            sfb_t<adcr_t, uint32_t, 8, 1> TRGEN;
            sfb_t<adcr_t, uint32_t, 11, 1> ADST;
        };

        union adcher_t {                            // A/D Channel Enable Register (ADCHER)
            __SFR(adcher_t, uint32_t, 0)
            sfb_t<adcher_t, uint32_t, 0, 1> CHEN0;
            sfb_t<adcher_t, uint32_t, 1, 1> CHEN1;
            sfb_t<adcher_t, uint32_t, 2, 1> CHEN2;
            sfb_t<adcher_t, uint32_t, 3, 1> CHEN3;
            sfb_t<adcher_t, uint32_t, 4, 1> CHEN4;
            sfb_t<adcher_t, uint32_t, 5, 1> CHEN5;
            sfb_t<adcher_t, uint32_t, 6, 1> CHEN6;
            sfb_t<adcher_t, uint32_t, 7, 1> CHEN7;
            sfb_t<adcher_t, uint32_t, 0, 8> CHEN;
            sfb_t<adcher_t, uint32_t, 8, 1> PRESEL;
        };

        union adcmpr_t {                            // A/D Compare Register (ADCMPR)
            __SFR(adcmpr_t, uint32_t, 0)
            sfb_t<adcmpr_t, uint32_t, 0, 1> CMPEN;
            sfb_t<adcmpr_t, uint32_t, 1, 1> CMPIE;
            sfb_t<adcmpr_t, uint32_t, 2, 1> CMPCOND;
            sfb_t<adcmpr_t, uint32_t, 3, 3> CMPCH;
            sfb_t<adcmpr_t, uint32_t, 8, 4> CMPMATCNT;
            sfb_t<adcmpr_t, uint32_t, 16, 10> CMPD;
        };

        union adsr_t {                              // A/D Status Register (ADSR)
            __SFR(adsr_t, uint32_t, 0)
            sfb_t<adsr_t, uint32_t, 0, 1> ADF;
            sfb_t<adsr_t, uint32_t, 1, 1> CMPF0;
            sfb_t<adsr_t, uint32_t, 2, 1> CMPF1;
            sfb_t<adsr_t, uint32_t, 3, 1> BUSY;
            sfb_t<adsr_t, uint32_t, 4, 3> CHANNEL;
            sfb_t<adsr_t, uint32_t, 8, 1> VALID;
            sfb_t<adsr_t, uint32_t, 16, 1> OVERRUN;
        };

        struct adc_t {
            const volatile sfr_t<addr_t> ADDR;          // A/D Data Registers (ADDR)
            uint32_t :32;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
            volatile sfr_t<adcr_t> ADCR;                // A/D Control Register (ADCR)
            volatile sfr_t<adcher_t> ADCHER;            // A/D Channel Enable Register (ADCHER)
            union {
                volatile sfr_t<adcmpr_t> ADCMPR[2];     // A/D Compare Register 0/1 (ADCMPR0/1)
                struct {
                    volatile sfr_t<adcmpr_t> ADCMPR0;   // A/D Compare Register 0 (ADCMPR0)
                    volatile sfr_t<adcmpr_t> ADCMPR1;   // A/D Compare Register 1 (ADCMPR1)
                };
            };
            volatile sfr_t<adsr_t> ADSR;                // A/D Status Register (ADSR)
        };

        extern adc_t ADC;   SFR_ADDR(ADC, ADC_ADDR);
    }
}

#endif  // __SFR_ADC_HPP
