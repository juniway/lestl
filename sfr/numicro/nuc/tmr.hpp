#ifndef __SFR_TMR_HPP
#define __SFR_TMR_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace tmr {                                 // Timer Control Registers
        enum {
            TMR0_IRQn = 8,
            TMR1_IRQn = 9,
            TMR2_IRQn = 10,
            TMR3_IRQn = 11
        };

        enum {
            TIMER0_ADDR = APB1_BASE + 0x10000,
            TIMER1_ADDR = APB1_BASE + 0x10020,
            TIMER2_ADDR = APB1_BASE + 0x10000,
            TIMER3_ADDR = APB1_BASE + 0x10020
        };

        union tcsr_t {                              // Timer Control Register (TCSR)
            __SFR(tcsr_t, uint32_t, 0)
            sfb_t<tcsr_t, 0, 8> PRESCALE;
            sfb_t<tcsr_t, 16, 1> TDR_EN;
            sfb_t<tcsr_t, 24, 1> CTB;
            sfb_t<tcsr_t, 25, 1> CACT;
            sfb_t<tcsr_t, 26, 1> CRST;
            sfb_t<tcsr_t, 27, 2> MODE;
            sfb_t<tcsr_t, 29, 1> IE;
            sfb_t<tcsr_t, 30, 1> CEN;
            sfb_t<tcsr_t, 31, 1> DBGACK_TMR;
        };

        union tisr_t {                              // Timer Interrupt Status Register (TISR)
            __SFR(tisr_t, uint32_t, 0)
            sfb_t<tisr_t, 0, 1> TIF;
        };

        struct tmr_t {
            volatile sfr_t<tcsr_t> TCSR;            // Timer Control Register (TCSR)
            volatile uint32_t TCMPR;                // Timer Compare Register (TCMPR)
            volatile sfr_t<tisr_t> TISR;            // Timer Interrupt Status Register (TISR)
            const volatile uint32_t TDR;            // Timer Data Register (TDR)
            uint64_t :64;
            uint64_t :64;
        };

        extern tmr_t TIMER0;        SFR_ADDR(TIMER0, TIMER0_ADDR);
        extern tmr_t TIMER1;        SFR_ADDR(TIMER1, TIMER1_ADDR);
        extern tmr_t TIMER01[2];    SFR_ADDR(TIMER01, TIMER0_ADDR);

        extern tmr_t TIMER2;        SFR_ADDR(TIMER2, TIMER2_ADDR);
        extern tmr_t TIMER3;        SFR_ADDR(TIMER3, TIMER3_ADDR);
        extern tmr_t TIMER23[2];    SFR_ADDR(TIMER23, TIMER2_ADDR);
    }
}

#endif  // __SFR_TMR_HPP
