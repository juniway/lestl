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
            sfb_t<addr_t, 0, 16> RSLT;
            sfb_t<addr_t, 16, 1> OVERRUN;
            sfb_t<addr_t, 17, 1> VALID;
        };

        union adcr_t {                              // A/D Control Register (ADCR)
            __SFR(adcr_t, uint32_t, 0)
            sfb_t<adcr_t, 0, 1> ADEN;
            sfb_t<adcr_t, 1, 1> ADIE;
            sfb_t<adcr_t, 2, 2> ADMD;
            sfb_t<adcr_t, 4, 2> TRGS;
            sfb_t<adcr_t, 6, 2> TRGCOND;
            sfb_t<adcr_t, 8, 1> TRGEN;
            sfb_t<adcr_t, 9, 1> PTEN;
            sfb_t<adcr_t, 10, 1> DIFFEN;
            sfb_t<adcr_t, 11, 1> ADST;
            sfb_t<adcr_t, 31, 1> DMOF;
        };

        union adcher_t {                            // A/D Channel Enable Register (ADCHER)
            __SFR(adcher_t, uint32_t, 0)
            sfb_t<adcher_t, 0, 1> CHEN0;
            sfb_t<adcher_t, 1, 1> CHEN1;
            sfb_t<adcher_t, 2, 1> CHEN2;
            sfb_t<adcher_t, 3, 1> CHEN3;
            sfb_t<adcher_t, 4, 1> CHEN4;
            sfb_t<adcher_t, 5, 1> CHEN5;
            sfb_t<adcher_t, 6, 1> CHEN6;
            sfb_t<adcher_t, 7, 1> CHEN7;
            sfb_t<adcher_t, 0, 8> CHEN;
            sfb_t<adcher_t, 8, 2> PRESEL;
        };

        union adcmpr_t {                            // A/D Compare Register (ADCMPR)
            __SFR(adcmpr_t, uint32_t, 0)
            sfb_t<adcmpr_t, 0, 1> CMPEN;
            sfb_t<adcmpr_t, 1, 1> CMPIE;
            sfb_t<adcmpr_t, 2, 1> CMPCOND;
            sfb_t<adcmpr_t, 3, 3> CMPCH;
            sfb_t<adcmpr_t, 8, 4> CMPMATCNT;
            sfb_t<adcmpr_t, 16, 12> CMPD;
        };

        union adsr_t {                              // A/D Status Register (ADSR)
            __SFR(adsr_t, uint32_t, 0)
            sfb_t<adsr_t, 0, 1> ADF;
            sfb_t<adsr_t, 1, 1> CMPF0;
            sfb_t<adsr_t, 2, 1> CMPF1;
            sfb_t<adsr_t, 3, 1> BUSY;
            sfb_t<adsr_t, 4, 3> CHANNEL;
            sfb_t<adsr_t, 8, 8> VALID;
            sfb_t<adsr_t, 16, 8> OVERRUN;
        };

        union adcalr_t {                            // A/D Calibration Register (ADCALR)
            __SFR(adcalr_t, uint32_t, 0)
            sfb_t<adcalr_t, 0, 1> CALEN;
            sfb_t<adcalr_t, 1, 1> CALDONE;
        };

        union adpdma_t {                            // A/D PDMA current transfer data Register (ADPDMA)
            __SFR(adpdma_t, uint32_t, 0)
            sfb_t<adpdma_t, 0, 12> AD_PDMA;
        };

        struct adc_t {
            union {
                const volatile sfr_t<addr_t> ADDR[8];   // A/D Data Registers (ADDR)
                struct {
                    const volatile sfr_t<addr_t> ADDR0; // A/D Data Register 0 (ADDR0)
                    const volatile sfr_t<addr_t> ADDR1; // A/D Data Register 1 (ADDR1)
                    const volatile sfr_t<addr_t> ADDR2; // A/D Data Register 2 (ADDR2)
                    const volatile sfr_t<addr_t> ADDR3; // A/D Data Register 3 (ADDR3)
                    const volatile sfr_t<addr_t> ADDR4; // A/D Data Register 4 (ADDR4)
                    const volatile sfr_t<addr_t> ADDR5; // A/D Data Register 5 (ADDR5)
                    const volatile sfr_t<addr_t> ADDR6; // A/D Data Register 6 (ADDR6)
                    const volatile sfr_t<addr_t> ADDR7; // A/D Data Register 7 (ADDR7)
                };
            };
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
            volatile sfr_t<adcalr_t> ADCALR;            // A/D Calibration Register (ADCALR)
            uint64_t :64;
            const volatile sfr_t<adpdma_t> ADPDMA;      // A/D PDMA current transfer data Register (ADPDMA)
        };

        extern adc_t ADC;   SFR_ADDR(ADC, ADC_ADDR);
    }
}

#endif  // __SFR_ADC_HPP
