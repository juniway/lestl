#ifndef __SFR_I2S_HPP
#define __SFR_I2S_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace i2s {                                 // I2S Interface Control Registers
        enum {
            I2S_IRQn = 27
        };

        enum {
            I2S_ADDR = APB2_BASE + 0xa0000
        };

        union con_t {                               // I2S Control Register (I2S_CON)
            __SFR(con_t, uint32_t, 0)
            sfb_t<con_t, 0, 1> I2SEN;
            sfb_t<con_t, 1, 1> TXEN;
            sfb_t<con_t, 2, 1> RXEN;
            sfb_t<con_t, 3, 1> MUTE;
            sfb_t<con_t, 4, 2> WORDWIDTH;
            sfb_t<con_t, 6, 1> MONO;
            sfb_t<con_t, 7, 1> FORMAT;
            sfb_t<con_t, 8, 1> SLAVE;
            sfb_t<con_t, 9, 3> TXTH;
            sfb_t<con_t, 12, 3> RXTH;
            sfb_t<con_t, 15, 1> MCLKEN;
            sfb_t<con_t, 16, 1> RCHZCEN;
            sfb_t<con_t, 17, 1> LCHZCEN;
            sfb_t<con_t, 18, 1> CLR_TXFIFO;
            sfb_t<con_t, 19, 1> CLR_RXFIFO;
            sfb_t<con_t, 20, 1> TXDMA;
            sfb_t<con_t, 21, 1> RXDMA;
        };

        union clkdiv_t {                            // I2S Clock Divider (I2S_CLKDIV)
            __SFR(clkdiv_t, uint32_t, 0)
            sfb_t<clkdiv_t, 0, 3> MCLK_DIV;
            sfb_t<clkdiv_t, 8, 8> BCLK_DIV;
        };

        union ie_t {                                // I2S Interrupt Enable Register (I2S_IE)
            __SFR(ie_t, uint32_t, 0)
            sfb_t<ie_t, 0, 1> RXUDFIE;
            sfb_t<ie_t, 1, 1> RXOVFIE;
            sfb_t<ie_t, 2, 1> RXTHIE;
            sfb_t<ie_t, 8, 1> TXUDFIE;
            sfb_t<ie_t, 9, 1> TXOVFIE;
            sfb_t<ie_t, 10, 1> TXTHIE;
            sfb_t<ie_t, 11, 1> RZCIE;
            sfb_t<ie_t, 12, 1> LZCIE;
        };

        union status_t {                            // I2S Status Register (I2S_STATUS)
            __SFR(status_t, uint32_t, 0)
            sfb_t<status_t, 0, 1> I2SINT;
            sfb_t<status_t, 1, 1> I2SRXINT;
            sfb_t<status_t, 2, 1> I2STXINT;
            sfb_t<status_t, 3, 1> RIGHT;
            sfb_t<status_t, 8, 1> RXUDF;
            sfb_t<status_t, 9, 1> RXOVF;
            sfb_t<status_t, 10, 1> RXTHF;
            sfb_t<status_t, 11, 1> RXFULL;
            sfb_t<status_t, 12, 1> RXEMPTY;
            sfb_t<status_t, 16, 1> TXUDF;
            sfb_t<status_t, 17, 1> TXOVF;
            sfb_t<status_t, 18, 1> TXTHF;
            sfb_t<status_t, 19, 1> TXFULL;
            sfb_t<status_t, 20, 1> TXEMPTY;
            sfb_t<status_t, 21, 1> TXBUSY;
            sfb_t<status_t, 22, 1> RZCF;
            sfb_t<status_t, 23, 1> LZCF;
            sfb_t<status_t, 24, 4> RX_LEVEL;
            sfb_t<status_t, 28, 4> TX_LEVEL;
        };

        struct i2s_t {
            volatile sfr_t<con_t> CON;              // I2S Control Register (I2S_CON)
            volatile sfr_t<clkdiv_t> CLKDIV;        // I2S Clock Divider (I2S_CLKDIV)
            volatile sfr_t<ie_t> IE;                // I2S Interrupt Enable Register (I2S_IE)
            volatile sfr_t<status_t> STATUS;        // I2S Status Register (I2S_STATUS)
            volatile uint32_t TXFIFO;               // I2S Transmit FIFO (I2S_TXFIFO)
            volatile uint32_t RXFIFO;               // I2S Receive FIFO (I2S_RXFIFO)
        };

        extern i2s_t I2S;   SFR_ADDR(I2S, I2S_ADDR);
    }
}

#endif  // __SFR_I2S_HPP
