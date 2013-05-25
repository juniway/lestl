#ifndef __SFR_GPIO_HPP
#define __SFR_GPIO_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace gpio {                                // General Purpose I/O Control Registers
        enum {
            EINT0_IRQn = 2,
            EINT1_IRQn = 3,
            GPIO_P0P1_IRQn = 4,
            GPIO_P2P3P4_IRQn = 5
        };

        enum {
            PORT0_ADDR = AHB_BASE + 0x4000,
            PORT1_ADDR = AHB_BASE + 0x4040,
            PORT2_ADDR = AHB_BASE + 0x4080,
            PORT3_ADDR = AHB_BASE + 0x40C0,
            PORT4_ADDR = AHB_BASE + 0x4100,
            DBNCECON_ADDR = AHB_BASE + 0x4180,
            PORT0_DOUT_ADDR = AHB_BASE + 0x4200,
            PORT1_DOUT_ADDR = AHB_BASE + 0x4220,
            PORT2_DOUT_ADDR = AHB_BASE + 0x4240,
            PORT3_DOUT_ADDR = AHB_BASE + 0x4260,
            PORT4_DOUT_ADDR = AHB_BASE + 0x4280,
        };

        union pmd_t {                               // GPIO Port I/O Mode Control (GPIO_PMD)
            __SFR(pmd_t, uint32_t, 0)
            sfb_t<pmd_t, 0, 2> PMD0;
            sfb_t<pmd_t, 2, 2> PMD1;
            sfb_t<pmd_t, 4, 2> PMD2;
            sfb_t<pmd_t, 6, 2> PMD3;
            sfb_t<pmd_t, 8, 2> PMD4;
            sfb_t<pmd_t, 10, 2> PMD5;
            sfb_t<pmd_t, 12, 2> PMD6;
            sfb_t<pmd_t, 14, 2> PMD7;
        };

        union offd_t {                              // GPIO Port Pin OFF Digital Resistor Enable (GPIO_OFFD)
            __SFR(offd_t, uint32_t, 0)
            sfb_t<offd_t, 16, 1> OFFD0;
            sfb_t<offd_t, 17, 1> OFFD1;
            sfb_t<offd_t, 18, 1> OFFD2;
            sfb_t<offd_t, 19, 1> OFFD3;
            sfb_t<offd_t, 20, 1> OFFD4;
            sfb_t<offd_t, 21, 1> OFFD5;
            sfb_t<offd_t, 22, 1> OFFD6;
            sfb_t<offd_t, 23, 1> OFFD7;
        };

        union dout_t {                              // GPIO Port Data Output Value (GPIO_DOUT)
            __SFR(dout_t, uint32_t, 0)
            sfb_t<dout_t, 0, 1> DOUT0;
            sfb_t<dout_t, 1, 1> DOUT1;
            sfb_t<dout_t, 2, 1> DOUT2;
            sfb_t<dout_t, 3, 1> DOUT3;
            sfb_t<dout_t, 4, 1> DOUT4;
            sfb_t<dout_t, 5, 1> DOUT5;
            sfb_t<dout_t, 6, 1> DOUT6;
            sfb_t<dout_t, 7, 1> DOUT7;
        };

        union dmask_t {                             // GPIO Port Data Output Write Mask (GPIO_DMASK)
            __SFR(dmask_t, uint32_t, 0)
            sfb_t<dmask_t, 0, 1> DMASK0;
            sfb_t<dmask_t, 1, 1> DMASK1;
            sfb_t<dmask_t, 2, 1> DMASK2;
            sfb_t<dmask_t, 3, 1> DMASK3;
            sfb_t<dmask_t, 4, 1> DMASK4;
            sfb_t<dmask_t, 5, 1> DMASK5;
            sfb_t<dmask_t, 6, 1> DMASK6;
            sfb_t<dmask_t, 7, 1> DMASK7;
        };

        union pin_t {                               // GPIO Port Pin Value (GPIO_PIN)
            __SFR(pin_t, uint32_t, 0)
            sfb_t<pin_t, 0, 1> PIN0;
            sfb_t<pin_t, 1, 1> PIN1;
            sfb_t<pin_t, 2, 1> PIN2;
            sfb_t<pin_t, 3, 1> PIN3;
            sfb_t<pin_t, 4, 1> PIN4;
            sfb_t<pin_t, 5, 1> PIN5;
            sfb_t<pin_t, 6, 1> PIN6;
            sfb_t<pin_t, 7, 1> PIN7;
        };

        union dben_t {                              // GPIO Port De-bounce Enable (GPIO_DBEN)
            __SFR(dben_t, uint32_t, 0)
            sfb_t<dben_t, 0, 1> DBEN0;
            sfb_t<dben_t, 1, 1> DBEN1;
            sfb_t<dben_t, 2, 1> DBEN2;
            sfb_t<dben_t, 3, 1> DBEN3;
            sfb_t<dben_t, 4, 1> DBEN4;
            sfb_t<dben_t, 5, 1> DBEN5;
            sfb_t<dben_t, 6, 1> DBEN6;
            sfb_t<dben_t, 7, 1> DBEN7;
        };

        union imd_t {                               // GPIO Port Interrupt Mode Control (GPIO_IMD)
            __SFR(imd_t, uint32_t, 0)
            sfb_t<imd_t, 0, 1> IMD0;
            sfb_t<imd_t, 1, 1> IMD1;
            sfb_t<imd_t, 2, 1> IMD2;
            sfb_t<imd_t, 3, 1> IMD3;
            sfb_t<imd_t, 4, 1> IMD4;
            sfb_t<imd_t, 5, 1> IMD5;
            sfb_t<imd_t, 6, 1> IMD6;
            sfb_t<imd_t, 7, 1> IMD7;
        };

        union ien_t {                               // GPIO Port Interrupt Enable Control (GPIO_IEN)
            __SFR(ien_t, uint32_t, 0)
            sfb_t<ien_t, 0, 1> IF_EN0;
            sfb_t<ien_t, 1, 1> IF_EN1;
            sfb_t<ien_t, 2, 1> IF_EN2;
            sfb_t<ien_t, 3, 1> IF_EN3;
            sfb_t<ien_t, 4, 1> IF_EN4;
            sfb_t<ien_t, 5, 1> IF_EN5;
            sfb_t<ien_t, 6, 1> IF_EN6;
            sfb_t<ien_t, 7, 1> IF_EN7;
            sfb_t<ien_t, 16, 1> IR_EN0;
            sfb_t<ien_t, 17, 1> IR_EN1;
            sfb_t<ien_t, 18, 1> IR_EN2;
            sfb_t<ien_t, 19, 1> IR_EN3;
            sfb_t<ien_t, 20, 1> IR_EN4;
            sfb_t<ien_t, 21, 1> IR_EN5;
            sfb_t<ien_t, 22, 1> IR_EN6;
            sfb_t<ien_t, 23, 1> IR_EN7;
        };

        union isrc_t {                              // GPIO Port Interrupt Trigger Source (GPIO_ISRC)
            __SFR(isrc_t, uint32_t, 0)
            sfb_t<isrc_t, 0, 1> ISRC0;
            sfb_t<isrc_t, 1, 1> ISRC1;
            sfb_t<isrc_t, 2, 1> ISRC2;
            sfb_t<isrc_t, 3, 1> ISRC3;
            sfb_t<isrc_t, 4, 1> ISRC4;
            sfb_t<isrc_t, 5, 1> ISRC5;
            sfb_t<isrc_t, 6, 1> ISRC6;
            sfb_t<isrc_t, 7, 1> ISRC7;
        };

        union dbncecon_t {                          // Interrupt De-bounce Cycle Control (DBNCECON)
            __SFR(dbncecon_t, uint32_t, 0)
            sfb_t<dbncecon_t, 0, 4> DBCLKSEL;
            sfb_t<dbncecon_t, 4, 1> DBCLKSRC;
            sfb_t<dbncecon_t, 5, 1> ICLK_ON;
        };

        struct gpio_t {
            volatile sfr_t<pmd_t> PMD;              // GPIO Port I/O Mode Control (GPIO_PMD)
            volatile sfr_t<offd_t> OFFD;            // GPIO Port Pin OFF Digital Resistor Enable (GPIO_OFFD)
            volatile sfr_t<dout_t> DOUT;            // GPIO Port Data Output Value (GPIO_DOUT)
            volatile sfr_t<dmask_t> DMASK;          // GPIO Port Data Output Write Mask (GPIO_DMASK)
            const volatile sfr_t<pin_t> PIN;        // GPIO Port Pin Value (GPIO_PIN)
            volatile sfr_t<dben_t> DBEN;            // GPIO Port De-bounce Enable (GPIO_DBEN)
            volatile sfr_t<imd_t> IMD;              // GPIO Port Interrupt Mode Control (GPIO_IMD)
            volatile sfr_t<ien_t> IEN;              // GPIO Port Interrupt Enable Control (GPIO_IEN)
            volatile sfr_t<isrc_t> ISRC;            // GPIO Port Interrupt Trigger Source (GPIO_ISRC)
            uint32_t :32;
            uint64_t :64;
            uint64_t :64;
            uint64_t :64;
        };

        extern gpio_t PORT0;    SFR_ADDR(PORT0, PORT0_ADDR);
        extern gpio_t PORT1;    SFR_ADDR(PORT1, PORT1_ADDR);
        extern gpio_t PORT2;    SFR_ADDR(PORT2, PORT2_ADDR);
        extern gpio_t PORT3;    SFR_ADDR(PORT3, PORT3_ADDR);
        extern gpio_t PORT4;    SFR_ADDR(PORT4, PORT4_ADDR);
        extern gpio_t PORT[5];  SFR_ADDR(PORT, PORT0_ADDR);
        extern volatile sfr_t<gpio::dbncecon_t> DBNCECON;       SFR_ADDR(DBNCECON, DBNCECON_ADDR);
        extern volatile uint32_t PORT0_DOUT[8];     SFR_ADDR(PORT0_DOUT, PORT0_DOUT_ADDR);
        extern volatile uint32_t PORT1_DOUT[8];     SFR_ADDR(PORT1_DOUT, PORT1_DOUT_ADDR);
        extern volatile uint32_t PORT2_DOUT[8];     SFR_ADDR(PORT2_DOUT, PORT2_DOUT_ADDR);
        extern volatile uint32_t PORT3_DOUT[8];     SFR_ADDR(PORT3_DOUT, PORT3_DOUT_ADDR);
        extern volatile uint32_t PORT4_DOUT[8];     SFR_ADDR(PORT4_DOUT, PORT4_DOUT_ADDR);
        extern volatile uint32_t PORT_DOUT[5][8];   SFR_ADDR(PORT_DOUT, PORT0_DOUT_ADDR);
    }
}

#endif  // __SFR_GPIO_HPP
