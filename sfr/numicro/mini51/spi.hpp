#ifndef __SFR_SPI_HPP
#define __SFR_SPI_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace spi {                                 // SPI Control Registers
        enum {
            SPI_IRQn = 14,
        };

        enum {
            SPI_ADDR = APB1_BASE + 0x30000
        };

        union cntrl_t {                             // SPI Control and Status Register (SPI_CNTRL)
            __SFR(cntrl_t, uint32_t, 0)
            sfb_t<cntrl_t, uint32_t, 0, 1> GO_BUSY;
            sfb_t<cntrl_t, uint32_t, 1, 1> RX_NEG;
            sfb_t<cntrl_t, uint32_t, 2, 1> TX_NEG;
            sfb_t<cntrl_t, uint32_t, 3, 5> TX_BIT_LEN;
            sfb_t<cntrl_t, uint32_t, 8, 2> TX_NUM;
            sfb_t<cntrl_t, uint32_t, 10, 1> LSB;
            sfb_t<cntrl_t, uint32_t, 11, 1> CLKP;
            sfb_t<cntrl_t, uint32_t, 12, 4> SP_CYCLE;
            sfb_t<cntrl_t, uint32_t, 16, 1> IF;
            sfb_t<cntrl_t, uint32_t, 17, 1> IE;
            sfb_t<cntrl_t, uint32_t, 18, 1> SLAVE;
            sfb_t<cntrl_t, uint32_t, 19, 2> REORDER;
            sfb_t<cntrl_t, uint32_t, 23, 1> VARCLK_EN;
        };

        union divider_t {                           // SPI Divider Register (SPI_DIVIDER)
            __SFR(divider_t, uint32_t, 0)
            sfb_t<divider_t, uint32_t, 0, 16> DIVIDER;
            sfb_t<divider_t, uint32_t, 16, 16> DIVIDER2;
        };

        union ssr_t {                               // SPI Slave Select Register (SPI_SSR)
            __SFR(ssr_t, uint32_t, 0)
            sfb_t<ssr_t, uint32_t, 0, 1> SSR;
            sfb_t<ssr_t, uint32_t, 2, 1> SS_LVL;
            sfb_t<ssr_t, uint32_t, 3, 1> ASS;
            sfb_t<ssr_t, uint32_t, 4, 1> SS_LTRIG;
            sfb_t<ssr_t, uint32_t, 5, 1> LTRIG_FLAG;
        };

        union cntrl2_t {
            __SFR(cntrl2_t, uint32_t, 0)
            sfb_t<cntrl2_t, uint32_t, 0, 1> DIV_ONE;
            sfb_t<cntrl2_t, uint32_t, 8, 1> NOSLVSEL;
            sfb_t<cntrl2_t, uint32_t, 9, 1> SLV_ABORT;
            sfb_t<cntrl2_t, uint32_t, 10, 1> SSTA_INTEN;
            sfb_t<cntrl2_t, uint32_t, 11, 1> SLV_START_INTSTS;
        };

        struct spi_t {
            volatile sfr_t<cntrl_t> CNTRL;          // SPI Control and Status Register (SPI_CNTRL)
            volatile sfr_t<divider_t> DIVIDER;      // SPI Divider Register (SPI_DIVIDER)
            volatile sfr_t<ssr_t> SSR;              // SPI Slave Select Register (SPI_SSR)
            uint32_t :32;
            union {                                 // SPI Data Receive Register (SPI_RX)
                const volatile uint32_t RX[2];
                struct {
                    const volatile uint32_t RX0;
                    const volatile uint32_t RX1;
                };
            };
            uint64_t :64;
            union {                                 // SPI Data Transmit Register (SPI_TX)
                volatile uint32_t TX[2];
                struct {
                    volatile uint32_t TX0;
                    volatile uint32_t TX1;
                };
            };
            uint64_t :64;
            uint32_t :32;
            volatile uint32_t VARCLK;               // SPI Variable Clock Pattern Register (SPI_VARCLK)
            volatile sfr_t<cntrl2_t> CNTRL2;
        };

        extern spi_t SPI;   SFR_ADDR(SPI, SPI_ADDR);
    }
}

#endif  // __SFR_SPI_HPP
