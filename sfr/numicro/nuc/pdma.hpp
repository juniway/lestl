#ifndef __SFR_PDMA_HPP
#define __SFR_PDMA_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace pdma {                                    // Peripheral DMA Control Registers
        enum {
            PDMA_IRQn = 26
        };

        enum {
            PDMA0_ADDR = AHB_BASE + 0x8000,
            PDMA1_ADDR = AHB_BASE + 0x8100,
            PDMA2_ADDR = AHB_BASE + 0x8200,
            PDMA3_ADDR = AHB_BASE + 0x8300,
            PDMA4_ADDR = AHB_BASE + 0x8400,
            PDMA5_ADDR = AHB_BASE + 0x8500,
            PDMA6_ADDR = AHB_BASE + 0x8600,
            PDMA7_ADDR = AHB_BASE + 0x8700,
            PDMA8_ADDR = AHB_BASE + 0x8800,
            PDMA9_ADDR = AHB_BASE + 0x8900,
            PDMA10_ADDR = AHB_BASE + 0x8A00,
            PDMA11_ADDR = AHB_BASE + 0x8B00,
            PDMA_GCR_ADDR = AHB_BASE + 0x8F00
        };

        union csr_t {                                   // PDMA Control and Status Register (PDMA_CSR)
            __SFR(csr_t, uint32_t, 0)
            sfb_t<csr_t, 0, 1> PDMACEN;
            sfb_t<csr_t, 1, 1> SW_RST;
            sfb_t<csr_t, 2, 2> MODE_SEL;
            sfb_t<csr_t, 4, 2> SAD_SEL;
            sfb_t<csr_t, 6, 2> DAD_SEL;
            sfb_t<csr_t, 19, 2> APB_TWS;
            sfb_t<csr_t, 23, 1> TRIG_EN;
        };

        union ier_t {                                   // PDMA Interrupt Enable Control Register (PDMA_IER)
            __SFR(ier_t, uint32_t, 0)
            sfb_t<ier_t, 0, 1> TABORT_IE;
            sfb_t<ier_t, 1, 1> BLKD_IE;
        };

        union isr_t {                                   // PDMA Interrupt Status Register (PDMA_ISR)
            __SFR(isr_t, uint32_t, 0)
            sfb_t<isr_t, 0, 1> TABORT_IF;
            sfb_t<isr_t, 1, 1> BLKD_IF;
        };

        union gcrcsr_t {                                // PDMA Global Control Register (PDMA_GCRCSR)
            __SFR(gcrcsr_t, uint32_t, 0)
            sfb_t<gcrcsr_t, 8, 1> CLK0_EN;
            sfb_t<gcrcsr_t, 9, 1> CLK1_EN;
            sfb_t<gcrcsr_t, 10, 1> CLK2_EN;
            sfb_t<gcrcsr_t, 11, 1> CLK3_EN;
            sfb_t<gcrcsr_t, 12, 1> CLK4_EN;
            sfb_t<gcrcsr_t, 13, 1> CLK5_EN;
            sfb_t<gcrcsr_t, 14, 1> CLK6_EN;
            sfb_t<gcrcsr_t, 15, 1> CLK7_EN;
        };

        union pdssr0_t {                                // PDMA Service Selection Control Register 0 (PDMA_PDSSR0)
            __SFR(pdssr0_t, uint32_t, 0)
            sfb_t<pdssr0_t, 0, 4> SPI0_RXSEL;
            sfb_t<pdssr0_t, 4, 4> SPI0_TXSEL;
            sfb_t<pdssr0_t, 8, 4> SPI1_RXSEL;
            sfb_t<pdssr0_t, 12, 4> SPI1_TXSEL;
        };

        union pdssr1_t {                                // PDMA Service Selection Control Register 1 (PDMA_PDSSR1)
            __SFR(pdssr1_t, uint32_t, 0)
            sfb_t<pdssr1_t, 0, 4> UART0_RXSEL;
            sfb_t<pdssr1_t, 4, 4> UART0_TXSEL;
            sfb_t<pdssr1_t, 8, 4> UART1_RXSEL;
            sfb_t<pdssr1_t, 12, 4> UART1_TXSEL;
            sfb_t<pdssr1_t, 24, 4> ADC_RXSEL;
        };

        union gcrisr_t {                                // PDMA Global Interrupt Status Register (PDMA_GCRISR)
            __SFR(gcrisr_t, uint32_t, 0)
            sfb_t<gcrisr_t, 0, 1> INTR0;
            sfb_t<gcrisr_t, 1, 1> INTR1;
            sfb_t<gcrisr_t, 2, 1> INTR2;
            sfb_t<gcrisr_t, 3, 1> INTR3;
            sfb_t<gcrisr_t, 4, 1> INTR4;
            sfb_t<gcrisr_t, 5, 1> INTR5;
            sfb_t<gcrisr_t, 6, 1> INTR6;
            sfb_t<gcrisr_t, 7, 1> INTR7;
            sfb_t<gcrisr_t, 8, 1> INTR8;
            sfb_t<gcrisr_t, 31, 1> INTR;
        };

        union pdssr2_t {                                // PDMA Service Selection Control Register 2 (PDMA_PDSSR2)
            __SFR(pdssr2_t, uint32_t, 0)
            sfb_t<pdssr2_t, 0, 4> I2S_RXSEL;
            sfb_t<pdssr2_t, 4, 4> I2S_TXSEL;
        };

        struct pdma_t {                             // Peripheral DMA Control Registers
            volatile sfr_t<csr_t> CSR;              // PDMA Control and Status Register (PDMA_CSR)
            void* volatile SAR;                     // PDMA Transfer Source Address Register (PDMA_SAR)
            void* volatile DAR;                     // PDMA Transfer Destination Address Register (PDMA_DAR)
            volatile uint16_t BCR;                  // PDMA Transfer Byte Count Register (PDMA_BCR)
            uint32_t :16;
            const volatile uint32_t POINT :4;       // PDMA Internal Buffer Pointer Register (PDMA_POINT)
            void* const volatile CSAR;              // PDMA Current Source Address Register (PDMA_CSAR)
            void* const volatile CDAR;              // PDMA Current Destination Address Register (PDMA_CDAR)
            const volatile uint16_t CBCR;           // PDMA Current Byte Count Register (PDMA_CBCR)
            volatile sfr_t<ier_t> IER;              // PDMA Interrupt Enable Control Register (PDMA_IER)
            volatile sfr_t<isr_t> ISR;              // PDMA Interrupt Status Register (PDMA_ISR)
            uint32_t __reserved1[22];
            const volatile uint32_t SBUF[4];        // PDMA Shared Buffer FIFO 0 (PDMA_SBUF)
            uint32_t __reserved2[28];
        };

        struct gcr_t {                              // Peripheral DMA Global Control Registers
            volatile sfr_t<gcrcsr_t> GCRCSR;        // PDMA Global Control Register (PDMA_GCRCSR)
            volatile sfr_t<pdssr0_t> PDSSR0;        // PDMA Service Selection Control Register 0 (PDMA_PDSSR0)
            volatile sfr_t<pdssr1_t> PDSSR1;        // PDMA Service Selection Control Register 1 (PDMA_PDSSR1)
            const volatile sfr_t<gcrisr_t> GCRISR;  // PDMA Global Interrupt Status Register (PDMA_GCRISR)
            volatile sfr_t<pdssr2_t> PDSSR2;        // PDMA Service Selection Control Register 2 (PDMA_PDSSR2)
        };

        extern pdma_t PDMA[];   SFR_ADDR(PDMA, PDMA0_ADDR);
        extern pdma_t PDMA0;    SFR_ADDR(PDMA0, PDMA0_ADDR);
        extern pdma_t PDMA1;    SFR_ADDR(PDMA1, PDMA1_ADDR);
        extern pdma_t PDMA2;    SFR_ADDR(PDMA2, PDMA2_ADDR);
        extern pdma_t PDMA3;    SFR_ADDR(PDMA3, PDMA3_ADDR);
        extern pdma_t PDMA4;    SFR_ADDR(PDMA4, PDMA4_ADDR);
        extern pdma_t PDMA5;    SFR_ADDR(PDMA5, PDMA5_ADDR);
        extern pdma_t PDMA6;    SFR_ADDR(PDMA6, PDMA6_ADDR);
        extern pdma_t PDMA7;    SFR_ADDR(PDMA7, PDMA7_ADDR);
        extern pdma_t PDMA8;    SFR_ADDR(PDMA8, PDMA8_ADDR);
        extern pdma_t PDMA9;    SFR_ADDR(PDMA9, PDMA9_ADDR);
        extern pdma_t PDMA10;   SFR_ADDR(PDMA10, PDMA10_ADDR);
        extern pdma_t PDMA11;   SFR_ADDR(PDMA11, PDMA11_ADDR);
        extern gcr_t PDMA_GCR;  SFR_ADDR(PDMA_GCR, PDMA_GCR_ADDR);
    }
}

#endif  // __SFR_PDMA_HPP
