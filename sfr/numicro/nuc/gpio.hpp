#ifndef __SFR_GPIO_HPP
#define __SFR_GPIO_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace gpio {                                // General Purpose I/O Control Registers
        enum {
            EINT0_IRQn = 2,
            EINT1_IRQn = 3,
            GPAB_IRQn = 4,
            GPCDE_IRQn = 5
        };

        enum {
            GPIOA_ADDR = AHB_BASE + 0x4000,
            GPIOB_ADDR = AHB_BASE + 0x4040,
            GPIOC_ADDR = AHB_BASE + 0x4080,
            GPIOD_ADDR = AHB_BASE + 0x40C0,
            GPIOE_ADDR = AHB_BASE + 0x4100,
            DBNCECON_ADDR = AHB_BASE + 0x4180,
            GPIOA_DOUT_ADDR = AHB_BASE + 0x4200,
            GPIOB_DOUT_ADDR = AHB_BASE + 0x4240,
            GPIOC_DOUT_ADDR = AHB_BASE + 0x4280,
            GPIOD_DOUT_ADDR = AHB_BASE + 0x42C0,
            GPIOE_DOUT_ADDR = AHB_BASE + 0x4300,
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
            sfb_t<pmd_t, 16, 2> PMD8;
            sfb_t<pmd_t, 18, 2> PMD9;
            sfb_t<pmd_t, 20, 2> PMD10;
            sfb_t<pmd_t, 22, 2> PMD11;
            sfb_t<pmd_t, 24, 2> PMD12;
            sfb_t<pmd_t, 26, 2> PMD13;
            sfb_t<pmd_t, 28, 2> PMD14;
            sfb_t<pmd_t, 30, 2> PMD15;
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
            sfb_t<offd_t, 24, 1> OFFD8;
            sfb_t<offd_t, 25, 1> OFFD9;
            sfb_t<offd_t, 26, 1> OFFD10;
            sfb_t<offd_t, 27, 1> OFFD11;
            sfb_t<offd_t, 28, 1> OFFD12;
            sfb_t<offd_t, 29, 1> OFFD13;
            sfb_t<offd_t, 30, 1> OFFD14;
            sfb_t<offd_t, 31, 1> OFFD15;
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
            sfb_t<dout_t, 8, 1> DOUT8;
            sfb_t<dout_t, 9, 1> DOUT9;
            sfb_t<dout_t, 10, 1> DOUT10;
            sfb_t<dout_t, 11, 1> DOUT11;
            sfb_t<dout_t, 12, 1> DOUT12;
            sfb_t<dout_t, 13, 1> DOUT13;
            sfb_t<dout_t, 14, 1> DOUT14;
            sfb_t<dout_t, 15, 1> DOUT15;
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
            sfb_t<dmask_t, 8, 1> DMASK8;
            sfb_t<dmask_t, 9, 1> DMASK9;
            sfb_t<dmask_t, 10, 1> DMASK10;
            sfb_t<dmask_t, 11, 1> DMASK11;
            sfb_t<dmask_t, 12, 1> DMASK12;
            sfb_t<dmask_t, 13, 1> DMASK13;
            sfb_t<dmask_t, 14, 1> DMASK14;
            sfb_t<dmask_t, 15, 1> DMASK15;
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
            sfb_t<pin_t, 8, 1> PIN8;
            sfb_t<pin_t, 9, 1> PIN9;
            sfb_t<pin_t, 10, 1> PIN10;
            sfb_t<pin_t, 11, 1> PIN11;
            sfb_t<pin_t, 12, 1> PIN12;
            sfb_t<pin_t, 13, 1> PIN13;
            sfb_t<pin_t, 14, 1> PIN14;
            sfb_t<pin_t, 15, 1> PIN15;
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
            sfb_t<dben_t, 8, 1> DBEN8;
            sfb_t<dben_t, 9, 1> DBEN9;
            sfb_t<dben_t, 10, 1> DBEN10;
            sfb_t<dben_t, 11, 1> DBEN11;
            sfb_t<dben_t, 12, 1> DBEN12;
            sfb_t<dben_t, 13, 1> DBEN13;
            sfb_t<dben_t, 14, 1> DBEN14;
            sfb_t<dben_t, 15, 1> DBEN15;
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
            sfb_t<imd_t, 8, 1> IMD8;
            sfb_t<imd_t, 9, 1> IMD9;
            sfb_t<imd_t, 10, 1> IMD10;
            sfb_t<imd_t, 11, 1> IMD11;
            sfb_t<imd_t, 12, 1> IMD12;
            sfb_t<imd_t, 13, 1> IMD13;
            sfb_t<imd_t, 14, 1> IMD14;
            sfb_t<imd_t, 15, 1> IMD15;
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
            sfb_t<ien_t, 8, 1> IF_EN8;
            sfb_t<ien_t, 9, 1> IF_EN9;
            sfb_t<ien_t, 10, 1> IF_EN10;
            sfb_t<ien_t, 11, 1> IF_EN11;
            sfb_t<ien_t, 12, 1> IF_EN12;
            sfb_t<ien_t, 13, 1> IF_EN13;
            sfb_t<ien_t, 14, 1> IF_EN14;
            sfb_t<ien_t, 15, 1> IF_EN15;
            sfb_t<ien_t, 16, 1> IR_EN0;
            sfb_t<ien_t, 17, 1> IR_EN1;
            sfb_t<ien_t, 18, 1> IR_EN2;
            sfb_t<ien_t, 19, 1> IR_EN3;
            sfb_t<ien_t, 20, 1> IR_EN4;
            sfb_t<ien_t, 21, 1> IR_EN5;
            sfb_t<ien_t, 22, 1> IR_EN6;
            sfb_t<ien_t, 23, 1> IR_EN7;
            sfb_t<ien_t, 24, 1> IR_EN8;
            sfb_t<ien_t, 25, 1> IR_EN9;
            sfb_t<ien_t, 26, 1> IR_EN10;
            sfb_t<ien_t, 27, 1> IR_EN11;
            sfb_t<ien_t, 28, 1> IR_EN12;
            sfb_t<ien_t, 29, 1> IR_EN13;
            sfb_t<ien_t, 30, 1> IR_EN14;
            sfb_t<ien_t, 31, 1> IR_EN15;
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
            sfb_t<isrc_t, 8, 1> ISRC8;
            sfb_t<isrc_t, 9, 1> ISRC9;
            sfb_t<isrc_t, 10, 1> ISRC10;
            sfb_t<isrc_t, 11, 1> ISRC11;
            sfb_t<isrc_t, 12, 1> ISRC12;
            sfb_t<isrc_t, 13, 1> ISRC13;
            sfb_t<isrc_t, 14, 1> ISRC14;
            sfb_t<isrc_t, 15, 1> ISRC15;
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

        extern gpio_t GPIOA;    SFR_ADDR(GPIOA, GPIOA_ADDR);
        extern gpio_t GPIOB;    SFR_ADDR(GPIOB, GPIOB_ADDR);
        extern gpio_t GPIOC;    SFR_ADDR(GPIOC, GPIOC_ADDR);
        extern gpio_t GPIOD;    SFR_ADDR(GPIOD, GPIOD_ADDR);
        extern gpio_t GPIOE;    SFR_ADDR(GPIOE, GPIOE_ADDR);
        extern gpio_t GPIO[5];  SFR_ADDR(GPIO, GPIOA_ADDR);
        extern volatile sfr_t<gpio::dbncecon_t> DBNCECON;       SFR_ADDR(DBNCECON, DBNCECON_ADDR);
        extern volatile uint32_t GPIOA_DOUT[16];        SFR_ADDR(GPIOA_DOUT, GPIOA_DOUT_ADDR);
        extern volatile uint32_t GPIOB_DOUT[16];        SFR_ADDR(GPIOB_DOUT, GPIOB_DOUT_ADDR);
        extern volatile uint32_t GPIOC_DOUT[16];        SFR_ADDR(GPIOC_DOUT, GPIOC_DOUT_ADDR);
        extern volatile uint32_t GPIOD_DOUT[16];        SFR_ADDR(GPIOD_DOUT, GPIOD_DOUT_ADDR);
        extern volatile uint32_t GPIOE_DOUT[16];        SFR_ADDR(GPIOE_DOUT, GPIOE_DOUT_ADDR);
        extern volatile uint32_t GPIO_DOUT[5][16];      SFR_ADDR(GPIO_DOUT, GPIOA_DOUT_ADDR);
    }
}

#endif  // __SFR_GPIO_HPP
