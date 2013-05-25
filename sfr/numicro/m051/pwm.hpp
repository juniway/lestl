#ifndef __SFR_PWM_HPP
#define __SFR_PWM_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace pwm {                                 // PWM Control Registers
        enum {
            PWMA_IRQn = 6,
            PWMB_IRQn = 7
        };

        enum {
            PWMA_ADDR = APB1_BASE + 0x40000,
            PWMB_ADDR = APB2_BASE + 0x40000
        };

        union ppr_t {                               // PWM Pre-Scale Register (PPR)
            __SFR(ppr_t, uint32_t, 0)
            sfb_t<ppr_t, 0, 8> CP01;
            sfb_t<ppr_t, 8, 8> CP23;
            sfb_t<ppr_t, 16, 8> DZI01;
            sfb_t<ppr_t, 24, 8> DZI23;
        };

        union csr_t {                               // PWM Clock Selector Register (CSR)
            __SFR(csr_t, uint32_t, 0)
            sfb_t<csr_t, 0, 3> CSR0;
            sfb_t<csr_t, 4, 3> CSR1;
            sfb_t<csr_t, 8, 3> CSR2;
            sfb_t<csr_t, 12, 3> CSR3;
        };

        union pcr_t {                               // PWM Control Register (PCR)
            __SFR(pcr_t, uint32_t, 0)
            sfb_t<pcr_t, 0, 1> CH0EN;
            sfb_t<pcr_t, 2, 1> CH0INV;
            sfb_t<pcr_t, 3, 1> CH0MOD;
            sfb_t<pcr_t, 4, 1> DZEN01;
            sfb_t<pcr_t, 5, 1> DZEN23;
            sfb_t<pcr_t, 8, 1> CH1EN;
            sfb_t<pcr_t, 10, 1> CH1INV;
            sfb_t<pcr_t, 11, 1> CH1MOD;
            sfb_t<pcr_t, 16, 1> CH2EN;
            sfb_t<pcr_t, 18, 1> CH2INV;
            sfb_t<pcr_t, 19, 1> CH2MOD;
            sfb_t<pcr_t, 24, 1> CH3EN;
            sfb_t<pcr_t, 26, 1> CH3INV;
            sfb_t<pcr_t, 27, 1> CH3MOD;
        };

        union pier_t {                              // PWM Interrupt Enable Register (PIER)
            __SFR(pier_t, uint32_t, 0)
            sfb_t<pier_t, 0, 1> PWMIE0;
            sfb_t<pier_t, 1, 1> PWMIE1;
            sfb_t<pier_t, 2, 1> PWMIE2;
            sfb_t<pier_t, 3, 1> PWMIE3;
        };

        union piir_t {                              // PWM Interrupt Indication Register (PIIR)
            __SFR(piir_t, uint32_t, 0)
            sfb_t<piir_t, 0, 1> PWMIF0;
            sfb_t<piir_t, 1, 1> PWMIF1;
            sfb_t<piir_t, 2, 1> PWMIF2;
            sfb_t<piir_t, 3, 1> PWMIF3;
        };

        union ccr0_t {                              // Capture Control Register (CCR0)
            __SFR(ccr0_t, uint32_t, 0)
            sfb_t<ccr0_t, 0, 1> INV0;
            sfb_t<ccr0_t, 1, 1> CRL_IE0;
            sfb_t<ccr0_t, 2, 1> CFL_IE0;
            sfb_t<ccr0_t, 3, 1> CAPCH0EN;
            sfb_t<ccr0_t, 4, 1> CAPIF0;
            sfb_t<ccr0_t, 6, 1> CRLRI0;
            sfb_t<ccr0_t, 7, 1> CFLRI0;
            sfb_t<ccr0_t, 16, 1> INV1;
            sfb_t<ccr0_t, 17, 1> CRL_IE1;
            sfb_t<ccr0_t, 18, 1> CFL_IE1;
            sfb_t<ccr0_t, 19, 1> CAPCH1EN;
            sfb_t<ccr0_t, 20, 1> CAPIF1;
            sfb_t<ccr0_t, 22, 1> CRLRI1;
            sfb_t<ccr0_t, 23, 1> CFLRI1;
        };

        union ccr2_t {                              // Capture Control Register (CCR2)
            __SFR(ccr2_t, uint32_t, 0)
            sfb_t<ccr2_t, 0, 1> INV2;
            sfb_t<ccr2_t, 1, 1> CRL_IE2;
            sfb_t<ccr2_t, 2, 1> CFL_IE2;
            sfb_t<ccr2_t, 3, 1> CAPCH2EN;
            sfb_t<ccr2_t, 4, 1> CAPIF2;
            sfb_t<ccr2_t, 6, 1> CRLRI2;
            sfb_t<ccr2_t, 7, 1> CFLRI2;
            sfb_t<ccr2_t, 16, 1> INV3;
            sfb_t<ccr2_t, 17, 1> CRL_IE3;
            sfb_t<ccr2_t, 18, 1> CFL_IE3;
            sfb_t<ccr2_t, 19, 1> CAPCH3EN;
            sfb_t<ccr2_t, 20, 1> CAPIF3;
            sfb_t<ccr2_t, 22, 1> CRLRI3;
            sfb_t<ccr2_t, 23, 1> CFLRI3;
        };

        union capenr_t {                            // Capture Input Enable Register (CAPENR)
            __SFR(capenr_t, uint32_t, 0)
            sfb_t<capenr_t, 0, 4> CAPENR;
            sfb_t<capenr_t, 0, 1> CAPEN0;
            sfb_t<capenr_t, 1, 1> CAPEN1;
            sfb_t<capenr_t, 2, 1> CAPEN2;
            sfb_t<capenr_t, 3, 1> CAPEN3;
        };

        union poe_t {                               // PWM Output Enable Register (POE)
            __SFR(poe_t, uint32_t, 0)
            sfb_t<poe_t, 0, 1> PWM0;
            sfb_t<poe_t, 1, 1> PWM1;
            sfb_t<poe_t, 2, 1> PWM2;
            sfb_t<poe_t, 3, 1> PWM3;
        };

        struct pwm_t {
            volatile sfr_t<ppr_t> PPR;              // PWM Pre-Scale Register (PPR)
            volatile sfr_t<csr_t> CSR;              // PWM Clock Selector Register (CSR)
            volatile sfr_t<pcr_t> PCR;              // PWM Control Register (PCR)
            volatile uint32_t CNR0;                 // PWM Counter Register 0 (CNR0)
            volatile uint32_t CMR0;                 // PWM Comparator Register 0 (CMR0)
            const volatile uint32_t PDR0;           // PWM Data Register 0 (PDR0)
            volatile uint32_t CNR1;                 // PWM Counter Register 0 (CNR1)
            volatile uint32_t CMR1;                 // PWM Comparator Register 0 (CMR1)
            const volatile uint32_t PDR1;           // PWM Data Register 0 (PDR1)
            volatile uint32_t CNR2;                 // PWM Counter Register 0 (CNR2)
            volatile uint32_t CMR2;                 // PWM Comparator Register 0 (CMR2)
            const volatile uint32_t PDR2;           // PWM Data Register 0 (PDR2)
            volatile uint32_t CNR3;                 // PWM Counter Register 0 (CNR3)
            volatile uint32_t CMR3;                 // PWM Comparator Register 0 (CMR3)
            const volatile uint32_t PDR3;           // PWM Data Register 0 (PDR3)
            uint32_t :32;
            volatile sfr_t<pier_t> PIER;            // PWM Interrupt Enable Register (PIER)
            volatile sfr_t<piir_t> PIIR;            // PWM Interrupt Indication Register (PIIR)
            uint64_t :64;
            volatile sfr_t<ccr0_t> CCR0;            // Capture Control Register (CCR0)
            volatile sfr_t<ccr2_t> CCR2;            // Capture Control Register (CCR2)
            const volatile uint32_t CRLR0;          // Capture Rising Latch Register 0 (CRLR0)
            const volatile uint32_t CFLR0;          // Capture Falling Latch Register 0 (CFLR0)
            const volatile uint32_t CRLR1;          // Capture Rising Latch Register 1 (CRLR1)
            const volatile uint32_t CFLR1;          // Capture Falling Latch Register 1 (CFLR1)
            const volatile uint32_t CRLR2;          // Capture Rising Latch Register 2 (CRLR2)
            const volatile uint32_t CFLR2;          // Capture Falling Latch Register 2 (CFLR2)
            const volatile uint32_t CRLR3;          // Capture Rising Latch Register 3 (CRLR3)
            const volatile uint32_t CFLR3;          // Capture Falling Latch Register 3 (CFLR3)
            volatile sfr_t<capenr_t> CAPENR;        // Capture Input Enable Register (CAPENR)
            volatile sfr_t<poe_t> POE;              // PWM Output Enable Register (POE)
        };

        extern pwm_t PWMA;  SFR_ADDR(PWMA, PWMA_ADDR);
        extern pwm_t PWMB;  SFR_ADDR(PWMB, PWMB_ADDR);
    }
}

#endif  // __SFR_PWM_HPP
