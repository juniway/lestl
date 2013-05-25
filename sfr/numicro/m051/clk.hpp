#ifndef __SFR_CLK_HPP
#define __SFR_CLK_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace clk {                                 // Clock Control Registers
        enum {
            PWRWU_IRQn = 28
        };

        enum {
            CLK_ADDR = AHB_BASE + 0x200
        };

        union pwrcon_t {                            // Power Down Control Register (PWRCON)
            __SFR(pwrcon_t, uint32_t, 0)
            sfb_t<pwrcon_t, 0, 1> XTL12M_EN;
            sfb_t<pwrcon_t, 2, 1> OSC22M_EN;
            sfb_t<pwrcon_t, 3, 1> OSC10K_EN;
            sfb_t<pwrcon_t, 4, 1> PD_WU_DLY;
            sfb_t<pwrcon_t, 5, 1> PD_WU_INT_EN;
            sfb_t<pwrcon_t, 6, 1> PD_WU_STS;
            sfb_t<pwrcon_t, 7, 1> PWR_DOWN;
            sfb_t<pwrcon_t, 8, 1> PD_WAIT_CPU;
        };

        union ahbclk_t {                            // AHB Devices Clock Enable Control Register (AHBCLK)
            __SFR(ahbclk_t, uint32_t, 0)
            sfb_t<ahbclk_t, 2, 1> ISP_EN;
            sfb_t<ahbclk_t, 3, 1> EBI_EN;
        };

        union apbclk_t {                            // APB Devices Clock Enable Control Register (APBCLK)
            __SFR(apbclk_t, uint32_t, 0)
            sfb_t<apbclk_t, 0, 1> WDT_EN;
            sfb_t<apbclk_t, 2, 1> TMR0_EN;
            sfb_t<apbclk_t, 3, 1> TMR1_EN;
            sfb_t<apbclk_t, 4, 1> TMR2_EN;
            sfb_t<apbclk_t, 5, 1> TMR3_EN;
            sfb_t<apbclk_t, 6, 1> FDIV_EN;
            sfb_t<apbclk_t, 8, 1> I2C_EN;
            sfb_t<apbclk_t, 12, 1> SPI0_EN;
            sfb_t<apbclk_t, 13, 1> SPI1_EN;
            sfb_t<apbclk_t, 16, 1> UART0_EN;
            sfb_t<apbclk_t, 17, 1> UART1_EN;
            sfb_t<apbclk_t, 20, 1> PWM01_EN;
            sfb_t<apbclk_t, 21, 1> PWM23_EN;
            sfb_t<apbclk_t, 22, 1> PWM45_EN;
            sfb_t<apbclk_t, 23, 1> PWM67_EN;
            sfb_t<apbclk_t, 28, 1> ADC_EN;
            sfb_t<apbclk_t, 30, 1> ACMP_EN;
        };

        union clkstatus_t {                         // Clock Status Register (CLKSTATUS)
            __SFR(clkstatus_t, uint32_t, 0)
            sfb_t<clkstatus_t, 0, 1> XTL12M_STB;
            sfb_t<clkstatus_t, 2, 1> PLL_STB;
            sfb_t<clkstatus_t, 3, 1> OSC10K_STB;
            sfb_t<clkstatus_t, 4, 1> OSC22M_STB;
            sfb_t<clkstatus_t, 7, 1> CLK_SW_FAIL;
        };

        union clksel0_t {                           // Clock Source Select Control Register 0 (CLKSEL0)
            __SFR(clksel0_t, uint32_t, 0)
            sfb_t<clksel0_t, 0, 3> HCLK_S;
            sfb_t<clksel0_t, 3, 3> STCLK_S;
        };

        union clksel1_t {                           // Clock Source Select Control Register 1 (CLKSEL1)
            __SFR(clksel1_t, uint32_t, 0)
            sfb_t<clksel1_t, 0, 2> WDT_S;
            sfb_t<clksel1_t, 2, 2> ADC_S;
            sfb_t<clksel1_t, 8, 3> TMR0_S;
            sfb_t<clksel1_t, 12, 3> TMR1_S;
            sfb_t<clksel1_t, 16, 3> TMR2_S;
            sfb_t<clksel1_t, 20, 3> TMR3_S;
            sfb_t<clksel1_t, 24, 2> UART_S;
            sfb_t<clksel1_t, 28, 2> PWM01_S;
            sfb_t<clksel1_t, 20, 2> PWM23_S;
        };

        union clkdiv_t {                            // Clock Divider Register (CLKDIV)
            __SFR(clkdiv_t, uint32_t, 0)
            sfb_t<clkdiv_t, 0, 4> HCLK_N;
            sfb_t<clkdiv_t, 8, 4> UART_N;
            sfb_t<clkdiv_t, 16, 4> ADC_N;
        };

        union clksel2_t {                           // Clock Source Select Control Register 2 (CLKSEL2)
            __SFR(clksel2_t, uint32_t, 0)
            sfb_t<clksel2_t, 2, 2> FRQDIV_S;
            sfb_t<clksel2_t, 4, 2> PWM45_S;
            sfb_t<clksel2_t, 6, 2> PWM67_S;
        };

        union pllcon_t {                            // PLL Control Register (PLLCON)
            __SFR(pllcon_t, uint32_t, 0)
            sfb_t<pllcon_t, 0, 9> FB_DV;
            sfb_t<pllcon_t, 9, 5> IN_DV;
            sfb_t<pllcon_t, 14, 2> OUT_DV;
            sfb_t<pllcon_t, 16, 1> PD;
            sfb_t<pllcon_t, 17, 1> BP;
            sfb_t<pllcon_t, 18, 1> OE;
            sfb_t<pllcon_t, 19, 1> PLL_SRC;
        };
        union frqdiv_t {                            // Frequency Divider Control Register (FRQDIV)
            __SFR(frqdiv_t, uint32_t, 0)
            sfb_t<frqdiv_t, 0, 4> FSEL;
            sfb_t<frqdiv_t, 4, 1> DIVIDER_EN;
        };

        struct clk_t {
            volatile sfr_t<pwrcon_t> PWRCON;        // Power Down Control Register (PWRCON)
            volatile sfr_t<ahbclk_t> AHBCLK;        // AHB Devices Clock Enable Control Register (AHBCLK)
            volatile sfr_t<apbclk_t> APBCLK;        // APB Devices Clock Enable Control Register (APBCLK)
            volatile sfr_t<clkstatus_t> CLKSTATUS;  // Clock Status Register (CLKSTATUS)
            volatile sfr_t<clksel0_t> CLKSEL0;      // Clock Source Select Control Register 0 (CLKSEL0)
            volatile sfr_t<clksel1_t> CLKSEL1;      // Clock Source Select Control Register 1 (CLKSEL1)
            volatile sfr_t<clkdiv_t> CLKDIV;        // Clock Divider Register (CLKDIV)
            volatile sfr_t<clksel2_t> CLKSEL2;      // Clock Source Select Control Register 2 (CLKSEL2)
            volatile sfr_t<pllcon_t> PLLCON;        // PLL Control Register (PLLCON)
            volatile sfr_t<frqdiv_t> FRQDIV;        // Frequency Divider Control Register (FRQDIV)
        };

        extern clk_t CLK;   SFR_ADDR(CLK, CLK_ADDR);
    }
}

#endif  // __SFR_CLK_HPP
