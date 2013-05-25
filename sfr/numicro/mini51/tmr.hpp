#ifndef __SFR_TMR_HPP
#define __SFR_TMR_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace tmr {                                 // Timer Control Registers
        enum {
            TMR0_IRQn = 8,
            TMR1_IRQn = 9
        };

        enum {
            TIMER0_ADDR = APB1_BASE + 0x10000,
            TIMER1_ADDR = APB1_BASE + 0x10020
        };

        union tcsr_t {                              // Timer Control Register (TCSR)
            __SFR(tcsr_t, uint32_t, 0)
            sfb_t<tcsr_t, uint32_t, 0, 8> PRESCALE;
            sfb_t<tcsr_t, uint32_t, 16, 1> TDR_EN;
            sfb_t<tcsr_t, uint32_t, 23, 1> WAKE_EN;
            sfb_t<tcsr_t, uint32_t, 24, 1> CTB;
            sfb_t<tcsr_t, uint32_t, 25, 1> CACT;
            sfb_t<tcsr_t, uint32_t, 26, 1> CRST;
            sfb_t<tcsr_t, uint32_t, 27, 2> MODE;
            sfb_t<tcsr_t, uint32_t, 29, 1> IE;
            sfb_t<tcsr_t, uint32_t, 30, 1> CEN;
            sfb_t<tcsr_t, uint32_t, 31, 1> DBGACK_TMR;
        };

        union tisr_t {                              // Timer Interrupt Status Register (TISR)
            __SFR(tisr_t, uint32_t, 0)
            sfb_t<tisr_t, uint32_t, 0, 1> TIF;
            sfb_t<tisr_t, uint32_t, 1, 1> TWF;
        };

        union texcon_t {
            __SFR(texcon_t, uint32_t, 0)
            sfb_t<texcon_t, uint32_t, 0, 1> TX_PHASE;
            sfb_t<texcon_t, uint32_t, 1, 2> TEX_EDGE;
            sfb_t<texcon_t, uint32_t, 3, 1> TEXEN;
            sfb_t<texcon_t, uint32_t, 4, 1> RSTCAPN;
            sfb_t<texcon_t, uint32_t, 5, 1> TEXIEN;
            sfb_t<texcon_t, uint32_t, 6, 1> TEXDB;
            sfb_t<texcon_t, uint32_t, 7, 1> TCDB;
            sfb_t<texcon_t, uint32_t, 8, 1> CAP_MODE;
        };

        union texisr_t {
            __SFR(texisr_t, uint32_t, 0)
            sfb_t<texisr_t, uint32_t, 0, 1> TEXIF;
        };

        struct tmr_t {
            volatile sfr_t<tcsr_t> TCSR;            // Timer Control Register (TCSR)
            volatile uint32_t TCMPR;                // Timer Compare Register (TCMPR)
            volatile sfr_t<tisr_t> TISR;            // Timer Interrupt Status Register (TISR)
            const volatile uint32_t TDR;            // Timer Data Register (TDR)
            const volatile uint32_t TCAP;
            volatile sfr_t<texcon_t> TEXCON;
            volatile sfr_t<texisr_t> TEXISR;
            uint32_t :32;
        };

        extern tmr_t TIMER0;        SFR_ADDR(TIMER0, TIMER0_ADDR);
        extern tmr_t TIMER1;        SFR_ADDR(TIMER1, TIMER1_ADDR);
        extern tmr_t TIMER01[2];    SFR_ADDR(TIMER01, TIMER0_ADDR);
    }
}

#endif  // __SFR_TMR_HPP
