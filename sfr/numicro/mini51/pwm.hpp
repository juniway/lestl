#ifndef __SFR_PWM_HPP
#define __SFR_PWM_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace pwm {                                 // PWM Control Registers
        enum {
            PWM_IRQn = 6
        };

        enum {
            PWM_ADDR = APB1_BASE + 0x40000
        };

        union ppr_t {                               // PWM Pre-Scale Register (PPR)
            __SFR(ppr_t, uint32_t, 0)
            sfb_t<ppr_t, 0, 8> CP01;
            sfb_t<ppr_t, 8, 8> CP23;
            sfb_t<ppr_t, 16, 8> CP45;
        };

        union csr_t {                               // PWM Clock Selector Register (CSR)
            __SFR(csr_t, uint32_t, 0)
            sfb_t<csr_t, 0, 3> CSR0;
            sfb_t<csr_t, 4, 3> CSR1;
            sfb_t<csr_t, 8, 3> CSR2;
            sfb_t<csr_t, 12, 3> CSR3;
            sfb_t<csr_t, 16, 3> CSR4;
            sfb_t<csr_t, 20, 3> CSR5;
        };

        union pcr_t {                               // PWM Control Register (PCR)
            __SFR(pcr_t, uint32_t, 0)
            sfb_t<pcr_t, 0, 1> CH0EN;
            sfb_t<pcr_t, 1, 1> DB_MODE;
            sfb_t<pcr_t, 2, 1> CH0INV;
            sfb_t<pcr_t, 3, 1> CH0MOD;
            sfb_t<pcr_t, 4, 1> CH1EN;
            sfb_t<pcr_t, 6, 1> CH1INV;
            sfb_t<pcr_t, 7, 1> CH1MOD;
            sfb_t<pcr_t, 8, 1> CH2EN;
            sfb_t<pcr_t, 10, 1> CH2INV;
            sfb_t<pcr_t, 11, 1> CH2MOD;
            sfb_t<pcr_t, 12, 1> CH3EN;
            sfb_t<pcr_t, 14, 1> CH3INV;
            sfb_t<pcr_t, 15, 1> CH3MOD;
            sfb_t<pcr_t, 16, 1> CH4EN;
            sfb_t<pcr_t, 18, 1> CH4INV;
            sfb_t<pcr_t, 19, 1> CH4MOD;
            sfb_t<pcr_t, 20, 1> CH5EN;
            sfb_t<pcr_t, 22, 1> CH5INV;
            sfb_t<pcr_t, 23, 1> CH5MOD;
            sfb_t<pcr_t, 24, 1> DZEN01;
            sfb_t<pcr_t, 25, 1> DZEN23;
            sfb_t<pcr_t, 26, 1> DZEN45;
            sfb_t<pcr_t, 27, 1> CLRPWM;
            sfb_t<pcr_t, 28, 2> PWMMOD;
            sfb_t<pcr_t, 30, 1> GRP;
            sfb_t<pcr_t, 31, 1> PWMTYPE;
        };

        union pier_t {                              // PWM Interrupt Enable Register (PIER)
            __SFR(pier_t, uint32_t, 0)
            sfb_t<pier_t, 0, 1> PWMPIE0;
            sfb_t<pier_t, 1, 1> PWMPIE1;
            sfb_t<pier_t, 2, 1> PWMPIE2;
            sfb_t<pier_t, 3, 1> PWMPIE3;
            sfb_t<pier_t, 4, 1> PWMPIE4;
            sfb_t<pier_t, 5, 1> PWMPIE5;
            sfb_t<pier_t, 0, 6> PWMPIE;
            sfb_t<pier_t, 8, 1> PWMDIE0;
            sfb_t<pier_t, 9, 1> PWMDIE1;
            sfb_t<pier_t, 10, 1> PWMDIE2;
            sfb_t<pier_t, 11, 1> PWMDIE3;
            sfb_t<pier_t, 12, 1> PWMDIE4;
            sfb_t<pier_t, 13, 1> PWMDIE5;
            sfb_t<pier_t, 8, 6> PWMDIE;
            sfb_t<pier_t, 16, 1> BRKIE;
            sfb_t<pier_t, 17, 1> INT_TYPE;
        };

        union piir_t {                              // PWM Interrupt Indication Register (PIIR)
            __SFR(piir_t, uint32_t, 0)
            sfb_t<piir_t, 0, 1> PWMPIF0;
            sfb_t<piir_t, 1, 1> PWMPIF1;
            sfb_t<piir_t, 2, 1> PWMPIF2;
            sfb_t<piir_t, 3, 1> PWMPIF3;
            sfb_t<pier_t, 4, 1> PWMPIF4;
            sfb_t<pier_t, 5, 1> PWMPIF5;
            sfb_t<piir_t, 0, 6> PWMPIF;
            sfb_t<pier_t, 8, 1> PWMDIF0;
            sfb_t<pier_t, 9, 1> PWMDIF1;
            sfb_t<pier_t, 10, 1> PWMDIF2;
            sfb_t<pier_t, 11, 1> PWMDIF3;
            sfb_t<pier_t, 12, 1> PWMDIF4;
            sfb_t<pier_t, 13, 1> PWMDIF5;
            sfb_t<pier_t, 8, 2> PWMDIF;
            sfb_t<pier_t, 16, 1> BKF0;
            sfb_t<pier_t, 17, 1> BKF1;
            sfb_t<pier_t, 16, 2> BKF;
        };

        union poe_t {                               // PWM Output Enable Register (POE)
            __SFR(poe_t, uint32_t, 0)
            sfb_t<poe_t, 0, 1> PWM0;
            sfb_t<poe_t, 1, 1> PWM1;
            sfb_t<poe_t, 2, 1> PWM2;
            sfb_t<poe_t, 3, 1> PWM3;
            sfb_t<poe_t, 4, 1> PWM4;
            sfb_t<poe_t, 5, 1> PWM5;
        };

        union pfbcon_t {
            __SFR(pfbcon_t, uint32_t, 0)
            sfb_t<pfbcon_t, 0, 1> BKEN0;
            sfb_t<pfbcon_t, 1, 1> BKEN1;
            sfb_t<pfbcon_t, 2, 1> CPO0BKEN;
            sfb_t<pfbcon_t, 7, 1> BKF;
            sfb_t<pfbcon_t, 24, 1> PWMBKO0;
            sfb_t<pfbcon_t, 25, 1> PWMBKO1;
            sfb_t<pfbcon_t, 26, 1> PWMBKO2;
            sfb_t<pfbcon_t, 27, 1> PWMBKO3;
            sfb_t<pfbcon_t, 28, 1> PWMBKO4;
            sfb_t<pfbcon_t, 29, 1> PWMBKO5;
            sfb_t<pfbcon_t, 24, 6> PWMBKO;
        };

        union pdzir_t {
            __SFR(pdzir_t, uint32_t, 0)
            sfb_t<pdzir_t, 0, 8> DZI01;
            sfb_t<pdzir_t, 8, 8> DZI23;
            sfb_t<pdzir_t, 16, 8> DZI45;
        };

        struct pwm_t {
            volatile sfr_t<ppr_t> PPR;              // PWM Pre-Scale Register (PPR)
            volatile sfr_t<csr_t> CSR;              // PWM Clock Selector Register (CSR)
            volatile sfr_t<pcr_t> PCR;              // PWM Control Register (PCR)
            volatile uint32_t CNR0;                 // PWM Counter Register 0 (CNR0)
            volatile uint32_t CNR1;                 // PWM Counter Register 1 (CNR1)
            volatile uint32_t CNR2;                 // PWM Counter Register 2 (CNR2)
            volatile uint32_t CNR3;                 // PWM Counter Register 3 (CNR3)
            volatile uint32_t CNR4;                 // PWM Counter Register 4 (CNR4)
            volatile uint32_t CNR5;                 // PWM Counter Register 5 (CNR5)
            volatile uint32_t CMR0;                 // PWM Comparator Register 0 (CMR0)
            volatile uint32_t CMR1;                 // PWM Comparator Register 1 (CMR1)
            volatile uint32_t CMR2;                 // PWM Comparator Register 2 (CMR2)
            volatile uint32_t CMR3;                 // PWM Comparator Register 3 (CMR3)
            volatile uint32_t CMR4;                 // PWM Comparator Register 4 (CMR4)
            volatile uint32_t CMR5;                 // PWM Comparator Register 5 (CMR5)
            uint32_t :32;
            uint32_t :32;
            uint32_t :32;
            uint32_t :32;
            uint32_t :32;
            uint32_t :32;
            volatile sfr_t<pier_t> PIER;            // PWM Interrupt Enable Register (PIER)
            volatile sfr_t<piir_t> PIIR;            // PWM Interrupt Indication Register (PIIR)
            volatile sfr_t<poe_t> POE;              // PWM Output Enable Register (POE)
            volatile sfr_t<pfbcon_t> PFBCON;
            volatile sfr_t<pdzir_t> PDZIR;
        };
        
        extern pwm_t PWM;   SFR_ADDR(PWM, PWM_ADDR);
    }
}

#endif  // __SFR_PWM_HPP
