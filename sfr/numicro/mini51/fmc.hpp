#ifndef __SFR_FMC_HPP
#define __SFR_FMC_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace fmc {                                 // Flash Memory Control Registers
        enum {
            FMC_ADDR = AHB_BASE + 0xC000
        };

        union ispcon_t {                            // ISP Control Register (ISPCON)
            __SFR(ispcon_t, uint32_t, 0)
            sfb_t<ispcon_t, 0, 1> ISPEN;
            sfb_t<ispcon_t, 1, 1> BS;
            sfb_t<ispcon_t, 4, 1> CFGUEN;
            sfb_t<ispcon_t, 5, 1> LDUEN;
            sfb_t<ispcon_t, 6, 1> ISPFF;
            sfb_t<ispcon_t, 7, 1> SWRST;
            sfb_t<ispcon_t, 8, 3> PT;
            sfb_t<ispcon_t, 12, 3> ET;
        };

        union ispcmd_t {                            // ISP Command Register (ISPCMD)
            __SFR(ispcmd_t, uint32_t, 0)
            sfb_t<ispcmd_t, 0, 4> FCTRL;
            sfb_t<ispcmd_t, 4, 1> FCEN;
            sfb_t<ispcmd_t, 5, 1> FOEN;
            sfb_t<ispcmd_t, 0, 6> ISPCMD;
        };

        union isptrg_t {                            // ISP Trigger Control Register (ISPTRG)
            __SFR(isptrg_t, uint32_t, 0)
            sfb_t<isptrg_t, 0, 1> ISPGO;
        };

        struct fmc_t {
            volatile sfr_t<ispcon_t> ISPCON;        // ISP Control Register (ISPCON)
            const void* volatile ISPADR;
            volatile uint32_t ISPDAT;               // ISP Data Register (ISPDAT)
            volatile sfr_t<ispcmd_t> ISPCMD;        // ISP Command Register (ISPCMD)
            volatile sfr_t<isptrg_t> ISPTRG;        // ISP Trigger Control Register (ISPTRG)
            void* const volatile DFBADR;            // Data Flash Base Address Register (DFBADR)
        };

        extern fmc_t FMC;   SFR_ADDR(FMC, FMC_ADDR);
    }
}

#endif  // __SFR_FMC_HPP
