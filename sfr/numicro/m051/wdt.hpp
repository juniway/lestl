#ifndef __SFR_WDT_HPP
#define __SFR_WDT_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace wdt {                                 // Watchdog Timer Control Registers
        enum {
            WDT_IRQn = 1
        };

        enum {
            WDT_ADDR = APB1_BASE + 0x4000
        };

        union wtcr_t {                              // Watchdog Timer Control Register (WTCR)
            __SFR(wtcr_t, uint32_t, 0)
            sfb_t<wtcr_t, 0, 1> WTR;
            sfb_t<wtcr_t, 1, 1> WTRE;
            sfb_t<wtcr_t, 2, 1> WTRF;
            sfb_t<wtcr_t, 3, 1> WTIF;
            sfb_t<wtcr_t, 4, 1> WTWKE;
            sfb_t<wtcr_t, 5, 1> WTWKF;
            sfb_t<wtcr_t, 6, 1> WTIE;
            sfb_t<wtcr_t, 7, 1> WTE;
            sfb_t<wtcr_t, 8, 3> WTIS;
        };

        struct wdt_t {
            volatile sfr_t<wtcr_t> WTCR;            // Watchdog Timer Control Register (WTCR)
        };

        extern wdt_t WDT;   SFR_ADDR(WDT, WDT_ADDR);
    }
}

#endif  // __SFR_WDT_HPP
