#ifndef __SFR_GCR_HPP
#define __SFR_GCR_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace gcr {                                 // System Manager Control Registers
        enum {
            BOD_IRQn = 0,
            HIRC_IRQn = 17
        };

        enum {
            GCR_ADDR = AHB_BASE
        };

        union rstsrc_t {                            // System Reset Source Register (RSTSRC)
            __SFR(rstsrc_t, uint32_t, 0)
            sfb_t<rstsrc_t, 0, 1> RSTS_POR;
            sfb_t<rstsrc_t, 1, 1> RSTS_RESET;
            sfb_t<rstsrc_t, 2, 1> RSTS_WDT;
            sfb_t<rstsrc_t, 4, 1> RSTS_BOD;
            sfb_t<rstsrc_t, 5, 1> RSTS_MCU;
            sfb_t<rstsrc_t, 7, 1> RSTS_CPU;
        };

        union iprstc1_t {                           // Peripheral Reset Control Register1 (IPRSTC1)
            __SFR(iprstc1_t, uint32_t, 0)
            sfb_t<iprstc1_t, 0, 1> CHIP_RST;
            sfb_t<iprstc1_t, 1, 1> CPU_RST;
        };

        union iprstc2_t {                           // Peripheral Reset Control Register2 (IPRSTC2)
            __SFR(iprstc2_t, uint32_t, 0)
            sfb_t<iprstc2_t, 1, 1> GPIO_RST;
            sfb_t<iprstc2_t, 2, 1> TMR0_RST;
            sfb_t<iprstc2_t, 3, 1> TMR1_RST;
            sfb_t<iprstc2_t, 8, 1> I2C_RST;
            sfb_t<iprstc2_t, 12, 1> SPI_RST;
            sfb_t<iprstc2_t, 16, 1> UART_RST;
            sfb_t<iprstc2_t, 20, 1> PWM_RST;
            sfb_t<iprstc2_t, 22, 1> ACMP_RST;
            sfb_t<iprstc2_t, 28, 1> ADC_RST;
        };

        union bodcr_t {                             // Brown-Out Detector Control Register (BODCR)
            __SFR(bodcr_t, uint32_t, 0)
            sfb_t<bodcr_t, 1, 2> BOD_VL;
            sfb_t<bodcr_t, 3, 1> BOD_RSTEN;
            sfb_t<bodcr_t, 4, 1> BOD_INTF;
            sfb_t<bodcr_t, 6, 1> BOD_OUT;
            sfb_t<bodcr_t, 16, 4> BOD27_TRIM;
            sfb_t<bodcr_t, 20, 4> BOD38_TRIM;
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

        union irctrimctl_t {
            __SFR(irctrimctl_t, uint32_t, 0)
            sfb_t<irctrimctl_t, 0, 1> TRIM_SEL;
            sfb_t<irctrimctl_t, 4, 2> TRIM_LOOP;
        };

        union irctrimien_t {
            __SFR(irctrimien_t, uint32_t, 0)
            sfb_t<irctrimien_t, 1, 1> TRIM_FAIL_IEN;
            sfb_t<irctrimien_t, 2, 1> ERR_32K_IEN;
        };

        union irctrimint_t {
            __SFR(irctrimint_t, uint32_t, 0)
            sfb_t<irctrimint_t, 0, 1> FREQ_LOCK;
            sfb_t<irctrimint_t, 1, 1> TRIM_FAIL_INT;
            sfb_t<irctrimint_t, 2, 1> ERR_32K_INT;
        };

        union regwrprot_t {                         // Register Write-Protection Control Register (REGWRPROT)
            __SFR(regwrprot_t, uint32_t, 0)
            const sfb_t<regwrprot_t, 0, 1> REGUNLOCK;
            sfb_t<regwrprot_t, 0, 8> REGWRPROT;
        };

        struct gcr_t {
            const volatile uint32_t PDID;           // Part Device ID Code Register (PDID)
            volatile sfr_t<rstsrc_t> RSTSRC;        // System Reset Source Register (RSTSRC)
            volatile sfr_t<iprstc1_t> IPRSTC1;      // Peripheral Reset Control Register1 (IPRSTC1)
            volatile sfr_t<iprstc2_t> IPRSTC2;      // Peripheral Reset Control Register2 (IPRSTC2)
            uint64_t :64;
            volatile sfr_t<bodcr_t> BODCR;          // Brown-Out Detector Control Register (BODCR)
            uint32_t :32;
            uint64_t :64;
            uint64_t :64;
            volatile sfr_t<mfp_t> P0_MFP;           // Multiple Function Pin P0 Control Register
            volatile sfr_t<mfp_t> P1_MFP;           // ... P1 ...
            volatile sfr_t<mfp_t> P2_MFP;           // ... P2 ...
            volatile sfr_t<mfp_t> P3_MFP;           // ... P3 ...
            volatile sfr_t<mfp_t> P4_MFP;           // ... P4 ...
            volatile sfr_t<mfp_t> P5_MFP;           // ... P5 ...
            uint32_t __reserved0[14];
            volatile sfr_t<irctrimctl_t> IRCTRIMCTL;
            volatile sfr_t<irctrimien_t> IRCTRIMIEN;
            volatile sfr_t<irctrimint_t> IRCTRIMINT;
            uint32_t __reserved1[29];
            volatile sfr_t<regwrprot_t> REGWRPROT;  // Register Write-Protection Control Register (REGWRPROT)
        };

        extern gcr_t GCR;   SFR_ADDR(GCR, GCR_ADDR);
        extern gcr_t SYS;   SFR_ADDR(SYS, GCR_ADDR);
    }

    namespace sys = gcr;
}

#endif  // __SFR_GCR_HPP
