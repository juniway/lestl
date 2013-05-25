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
            sfb_t<iprstc1_t, 2, 1> PDMA_RST;
        };

        union iprstc2_t {                           // Peripheral Reset Control Register2 (IPRSTC2)
            __SFR(iprstc2_t, uint32_t, 0)
            sfb_t<iprstc2_t, 1, 1> GPIO_RST;
            sfb_t<iprstc2_t, 2, 1> TMR0_RST;
            sfb_t<iprstc2_t, 3, 1> TMR1_RST;
            sfb_t<iprstc2_t, 4, 1> TMR2_RST;
            sfb_t<iprstc2_t, 5, 1> TMR3_RST;
            sfb_t<iprstc2_t, 8, 1> I2C0_RST;
            sfb_t<iprstc2_t, 9, 1> I2C1_RST;
            sfb_t<iprstc2_t, 12, 1> SPI0_RST;
            sfb_t<iprstc2_t, 13, 1> SPI1_RST;
            sfb_t<iprstc2_t, 16, 1> UART0_RST;
            sfb_t<iprstc2_t, 17, 1> UART1_RST;
            sfb_t<iprstc2_t, 20, 1> PWM03_RST;
            sfb_t<iprstc2_t, 21, 1> PWM47_RST;
            sfb_t<iprstc2_t, 22, 1> ACMP_RST;
            sfb_t<iprstc2_t, 27, 1> USBD_RST;
            sfb_t<iprstc2_t, 28, 1> ADC_RST;
            sfb_t<iprstc2_t, 29, 1> I2S_RST;
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

        union gpamfp_t {                            // Multiple Function Pin GPIOA Control Register (GPA_MFP)
            __SFR(gpamfp_t, uint32_t, 0)
            sfb_t<gpamfp_t, 0, 1> GPA_MFP0;
            sfb_t<gpamfp_t, 1, 1> GPA_MFP1;
            sfb_t<gpamfp_t, 2, 1> GPA_MFP2;
            sfb_t<gpamfp_t, 3, 1> GPA_MFP3;
            sfb_t<gpamfp_t, 4, 1> GPA_MFP4;
            sfb_t<gpamfp_t, 5, 1> GPA_MFP5;
            sfb_t<gpamfp_t, 6, 1> GPA_MFP6;
            sfb_t<gpamfp_t, 7, 1> GPA_MFP7;
            sfb_t<gpamfp_t, 8, 1> GPA_MFP8;
            sfb_t<gpamfp_t, 9, 1> GPA_MFP9;
            sfb_t<gpamfp_t, 10, 1> GPA_MFP10;
            sfb_t<gpamfp_t, 11, 1> GPA_MFP11;
            sfb_t<gpamfp_t, 12, 1> GPA_MFP12;
            sfb_t<gpamfp_t, 13, 1> GPA_MFP13;
            sfb_t<gpamfp_t, 14, 1> GPA_MFP14;
            sfb_t<gpamfp_t, 15, 1> GPA_MFP15;
            sfb_t<gpamfp_t, 16, 16> SCHMITT;
        };

        union gpbmfp_t {                            // Multiple Function Pin GPIOB Control Register (GPB_MFP)
            __SFR(gpbmfp_t, uint32_t, 0)
            sfb_t<gpbmfp_t, 0, 1> GPB_MFP0;
            sfb_t<gpbmfp_t, 1, 1> GPB_MFP1;
            sfb_t<gpbmfp_t, 2, 1> GPB_MFP2;
            sfb_t<gpbmfp_t, 3, 1> GPB_MFP3;
            sfb_t<gpbmfp_t, 4, 1> GPB_MFP4;
            sfb_t<gpbmfp_t, 5, 1> GPB_MFP5;
            sfb_t<gpbmfp_t, 6, 1> GPB_MFP6;
            sfb_t<gpbmfp_t, 7, 1> GPB_MFP7;
            sfb_t<gpbmfp_t, 8, 1> GPB_MFP8;
            sfb_t<gpbmfp_t, 9, 1> GPB_MFP9;
            sfb_t<gpbmfp_t, 10, 1> GPB_MFP10;
            sfb_t<gpbmfp_t, 11, 1> GPB_MFP11;
            sfb_t<gpbmfp_t, 12, 1> GPB_MFP12;
            sfb_t<gpbmfp_t, 13, 1> GPB_MFP13;
            sfb_t<gpbmfp_t, 14, 1> GPB_MFP14;
            sfb_t<gpbmfp_t, 15, 1> GPB_MFP15;
            sfb_t<gpbmfp_t, 16, 16> SCHMITT;
        };

        union gpcmfp_t {                            // Multiple Function Pin GPIOC Control Register (GPC_MFP)
            __SFR(gpcmfp_t, uint32_t, 0)
            sfb_t<gpcmfp_t, 0, 1> GPC_MFP0;
            sfb_t<gpcmfp_t, 1, 1> GPC_MFP1;
            sfb_t<gpcmfp_t, 2, 1> GPC_MFP2;
            sfb_t<gpcmfp_t, 3, 1> GPC_MFP3;
            sfb_t<gpcmfp_t, 6, 1> GPC_MFP6;
            sfb_t<gpcmfp_t, 7, 1> GPC_MFP7;
            sfb_t<gpcmfp_t, 8, 1> GPC_MFP8;
            sfb_t<gpcmfp_t, 9, 1> GPC_MFP9;
            sfb_t<gpcmfp_t, 10, 1> GPC_MFP10;
            sfb_t<gpcmfp_t, 11, 1> GPC_MFP11;
            sfb_t<gpcmfp_t, 14, 1> GPC_MFP14;
            sfb_t<gpcmfp_t, 15, 1> GPC_MFP15;
            sfb_t<gpcmfp_t, 6, 16> SCHMITT;
        };

        union gpemfp_t {                            // Multiple Function Pin GPIOE Control Register (GPE_MFP)
            __SFR(gpemfp_t, uint32_t, 0)
            sfb_t<gpemfp_t, 5, 1> GPE_MFP5;
            sfb_t<gpemfp_t, 16, 16> SCHMITT;
        };

        union altmfp_t {                            // Alternative Multiple Function Pin Control Register (ALT_MFP)
            __SFR(altmfp_t, uint32_t, 0)
            sfb_t<altmfp_t, 4, 1> PB11_PWM4;        // GPB11
            sfb_t<altmfp_t, 5, 1> PC0_I2SLRCLK; // GPC0
            sfb_t<altmfp_t, 6, 1> PC1_I2SBCLK;  // GPC1
            sfb_t<altmfp_t, 7, 1> PC2_I2SDI;        // GPC2
            sfb_t<altmfp_t, 8, 1> PC3_I2SDO;        // GPC3
            sfb_t<altmfp_t, 9, 1> PA15_I2SMCLK; // GPA15
            sfb_t<altmfp_t, 10, 1> PB12_CLKO;       // GPB12
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
            volatile sfr_t<gpamfp_t> GPAMFP;        // Multiple Function Pin GPIOA Control Register (GPA_MFP)
            volatile sfr_t<gpbmfp_t> GPBMFP;        // ... GPIOB ... (GPB_MFP)
            volatile sfr_t<gpcmfp_t> GPCMFP;        // ... GPIOC ... (GPC_MFP)
            uint32_t :32;
            volatile sfr_t<gpemfp_t> GPEMFP;        // ... GPIOE ... (GPE_MFP)
            uint32_t :32;
            uint64_t :64;
            volatile sfr_t<altmfp_t> ALTMFP;        // Alternative Multiple Function Pin Control Register (ALT_MFP)
            uint32_t __reserved[43];
            volatile sfr_t<regwrprot_t> REGWRPROT;  // Register Write-Protection Control Register (REGWRPROT)
        };

        extern gcr_t GCR;   SFR_ADDR(GCR, GCR_ADDR);
        extern gcr_t SYS;   SFR_ADDR(SYS, GCR_ADDR);
    }

    namespace sys = gcr;
}

#endif  // __SFR_GCR_HPP
