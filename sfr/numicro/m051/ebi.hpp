#ifndef __SFR_EBI_HPP
#define __SFR_EBI_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace ebi {                                 // EBI Controller Registers
        enum {
            EBI_ADDR = AHB_BASE + 0x10000
        };

        union ebicon_t {                            // External Bus Interface Control Register (EBICON)
            __SFR(ebicon_t, uint32_t, 0)
            sfb_t<ebicon_t, 0, 1> EXTEN;
            sfb_t<ebicon_t, 1, 1> EXTBW16;
            sfb_t<ebicon_t, 8, 3> MCLKDIV;
            sfb_t<ebicon_t, 16, 3> EXTTALE;
        };

        union extime_t {                            // External Bus Interface Timing Control Register (EXTIME)
            __SFR(extime_t, uint32_t, 0)
            sfb_t<extime_t, 3, 5> EXTTACC;
            sfb_t<extime_t, 8, 3> EXTTAHD;
            sfb_t<extime_t, 12, 4> EXTIW2X;
            sfb_t<extime_t, 24, 4> EXTIR2R;
        };

        struct ebi_t {
            volatile sfr_t<ebicon_t> EBICON;        // External Bus Interface Control Register (EBICON)
            volatile sfr_t<extime_t> EXTIME;        // External Bus Interface Timing Control Register (EXTIME)
        };

        extern ebi_t EBI;   SFR_ADDR(EBI, EBI_ADDR);
    }
}

#endif  // __SFR_EBI_HPP
