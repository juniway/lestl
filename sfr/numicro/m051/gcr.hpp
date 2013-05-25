#ifndef __SFR_GCR_HPP
#define __SFR_GCR_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace gcr {                                 // System Manager Control Registers
        enum {
            BOD_IRQn = 0
        };

        enum {
            GCR_ADDR = AHB_BASE
        };

        union rstsrc_t {                            // System Reset Source Register (RSTSRC)
            __SFR(rstsrc_t, uint32_t, 0)
            sfb_t<rstsrc_t, 0, 1> RSTS_POR;
            sfb_t<rstsrc_t, 1, 1> RSTS_RESET;
            sfb_t<rstsrc_t, 2, 1> RSTS_WDT;
            sfb_t<rstsrc_t, 3, 1> RSTS_LVR;
            sfb_t<rstsrc_t, 4, 1> RSTS_BOD;
            sfb_t<rstsrc_t, 5, 1> RSTS_SYS;
            sfb_t<rstsrc_t, 7, 1> RSTS_CPU;
        };

        union iprstc1_t {                           // Peripheral Reset Control Register1 (IPRSTC1)
            __SFR(iprstc1_t, uint32_t, 0)
            sfb_t<iprstc1_t, 0, 1> CHIP_RST;
            sfb_t<iprstc1_t, 1, 1> CPU_RST;
            sfb_t<iprstc1_t, 3, 1> EBI_RST;
        };

        union iprstc2_t {                           // Peripheral Reset Control Register2 (IPRSTC2)
            __SFR(iprstc2_t, uint32_t, 0)
            sfb_t<iprstc2_t, 1, 1> GPIO_RST;
            sfb_t<iprstc2_t, 2, 1> TMR0_RST;
            sfb_t<iprstc2_t, 3, 1> TMR1_RST;
            sfb_t<iprstc2_t, 4, 1> TMR2_RST;
            sfb_t<iprstc2_t, 5, 1> TMR3_RST;
            sfb_t<iprstc2_t, 8, 1> I2C_RST;
            sfb_t<iprstc2_t, 12, 1> SPI0_RST;
            sfb_t<iprstc2_t, 13, 1> SPI1_RST;
            sfb_t<iprstc2_t, 16, 1> UART0_RST;
            sfb_t<iprstc2_t, 17, 1> UART1_RST;
            sfb_t<iprstc2_t, 20, 1> PWM03_RST;
            sfb_t<iprstc2_t, 21, 1> PWM47_RST;
            sfb_t<iprstc2_t, 22, 1> ACMP_RST;
            sfb_t<iprstc2_t, 28, 1> ADC_RST;
        };

        union bodcr_t {                             // Brown-Out Detector Control Register (BODCR)
            __SFR(bodcr_t, uint32_t, 0)
            sfb_t<bodcr_t, 0, 1> BOD_EN;
            sfb_t<bodcr_t, 1, 2> BOD_VL;
            sfb_t<bodcr_t, 3, 1> BOD_RSTEN;
            sfb_t<bodcr_t, 4, 1> BOD_INTF;
            sfb_t<bodcr_t, 5, 1> BOD_LPM;
            sfb_t<bodcr_t, 6, 1> BOD_OUT;
            sfb_t<bodcr_t, 7, 1> LVR_EN;
        };

        union tempcr_t {                            // Temperature Sensor Control Register (TEMPCR)
            __SFR(tempcr_t, uint32_t, 0)
            sfb_t<tempcr_t, 0, 1> VTEMP_EN;
        };

        union porcr_t {                             // Power-On-Reset Control Register (PORCR)
            __SFR(porcr_t, uint32_t, 0)
            sfb_t<porcr_t, 0, 16> POR_DIS_CODE;
        };

        union mfp_t {                           // Multiple Function Pin Control Register (Px_MFP)
            __SFR(mfp_t, uint32_t, 0)
            sfb_t<mfp_t, 0, 1> MFP0;
            sfb_t<mfp_t, 1, 1> MFP1;
            sfb_t<mfp_t, 2, 1> MFP2;
            sfb_t<mfp_t, 3, 1> MFP3;
            sfb_t<mfp_t, 4, 1> MFP4;
            sfb_t<mfp_t, 5, 1> MFP5;
            sfb_t<mfp_t, 6, 1> MFP6;
            sfb_t<mfp_t, 7, 1> MFP7;
            sfb_t<mfp_t, 0, 8> MFP;
            sfb_t<mfp_t, 8, 1> ALT0;
            sfb_t<mfp_t, 9, 1> ALT1;
            sfb_t<mfp_t, 10, 1> ALT2;
            sfb_t<mfp_t, 11, 1> ALT3;
            sfb_t<mfp_t, 12, 1> ALT4;
            sfb_t<mfp_t, 13, 1> ALT5;
            sfb_t<mfp_t, 14, 1> ALT6;
            sfb_t<mfp_t, 15, 1> ALT7;
            sfb_t<mfp_t, 8, 8> ALT;
            sfb_t<mfp_t, 16, 1> SCHMITT0;
            sfb_t<mfp_t, 17, 1> SCHMITT1;
            sfb_t<mfp_t, 18, 1> SCHMITT2;
            sfb_t<mfp_t, 19, 1> SCHMITT3;
            sfb_t<mfp_t, 20, 1> SCHMITT4;
            sfb_t<mfp_t, 21, 1> SCHMITT5;
            sfb_t<mfp_t, 22, 1> SCHMITT6;
            sfb_t<mfp_t, 23, 1> SCHMITT7;
            sfb_t<mfp_t, 16, 8> SCHMITT;
        };

        union regwrprot_t {                         // Register Write-Protection Control Register (REGWRPROT)
            __SFR(regwrprot_t, uint32_t, 0)
            sfb_t<regwrprot_t, 0, 1> REGPROTDIS;
            sfb_t<regwrprot_t, 0, 8> REGWRPROT;
        };

        struct gcr_t {
            const volatile uint32_t PDID;           // Part Device ID Code Register (PDID)
            volatile sfr_t<rstsrc_t> RSTSRC;        // System Reset Source Register (RSTSRC)
            volatile sfr_t<iprstc1_t> IPRSTC1;      // Peripheral Reset Control Register1 (IPRSTC1)
            volatile sfr_t<iprstc2_t> IPRSTC2;      // Peripheral Reset Control Register2 (IPRSTC2)
            uint64_t :64;
            volatile sfr_t<bodcr_t> BODCR;          // Brown-Out Detector Control Register (BODCR)
            volatile sfr_t<tempcr_t> TEMPCR;        // Temperature Sensor Control Register (TEMPCR)
            uint32_t :32;
            volatile sfr_t<porcr_t> PORCR;          // Power-On-Reset Control Register (PORCR)
            uint64_t :64;
            volatile sfr_t<mfp_t> P0_MFP;           // Multiple Function Port0 Control Register (P0_MFP)
            volatile sfr_t<mfp_t> P1_MFP;           // ... Port1 ... (P1_MFP)
            volatile sfr_t<mfp_t> P2_MFP;           // ... Port2 ... (P2_MFP)
            volatile sfr_t<mfp_t> P3_MFP;           // ... Port3 ... (P3_MFP)
            volatile sfr_t<mfp_t> P4_MFP;           // ... Port4 ... (P4_MFP)
            uint32_t __reserved[47];
            volatile sfr_t<regwrprot_t> REGWRPROT;  // Register Write-Protection Control Register (REGWRPROT)
        };

        extern gcr_t GCR;   SFR_ADDR(GCR, GCR_ADDR);
        extern gcr_t SYS;   SFR_ADDR(SYS, GCR_ADDR);
    }

    namespace sys = gcr;
}

#endif  // __SFR_GCR_HPP
