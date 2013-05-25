#ifndef __SFR_SYSINT_HPP
#define __SFR_SYSINT_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace sysint {                              // Interrupt Source Control Registers
        enum {
            SYSINT_ADDR = AHB_BASE + 0x300
        };

        union irq0src_t {                           // Interrupt Source Identity Register (IRQ0_SRC)
            __SFR(irq0src_t, uint32_t, 0)
            sfb_t<irq0src_t, 0, 1> BOD_INT;
        };

        union irq1src_t {                           // Interrupt Source Identity Register (IRQ1_SRC)
            __SFR(irq1src_t, uint32_t, 0)
            sfb_t<irq1src_t, 0, 1> WDT_INT;
        };

        union irq2src_t {                           // Interrupt Source Identity Register (IRQ2_SRC)
            __SFR(irq2src_t, uint32_t, 0)
            sfb_t<irq2src_t, 0, 1> EINT0;
        };

        union irq3src_t {                           // Interrupt Source Identity Register (IRQ3_SRC)
            __SFR(irq3src_t, uint32_t, 0)
            sfb_t<irq3src_t, 0, 1> EINT1;
        };

        union irq4src_t {                           // Interrupt Source Identity Register (IRQ4_SRC)
            __SFR(irq4src_t, uint32_t, 0)
            sfb_t<irq4src_t, 0, 1> GPA_INT;
            sfb_t<irq4src_t, 1, 1> GPB_INT;
        };

        union irq5src_t {                           // Interrupt Source Identity Register (IRQ5_SRC)
            __SFR(irq5src_t, uint32_t, 0)
            sfb_t<irq5src_t, 0, 1> GPC_INT;
            sfb_t<irq5src_t, 2, 1> GPE_INT;
        };

        union irq6src_t {                           // Interrupt Source Identity Register (IRQ6_SRC)
            __SFR(irq6src_t, uint32_t, 0)
            sfb_t<irq6src_t, 0, 1> PWM0_INT;
            sfb_t<irq6src_t, 1, 1> PWM1_INT;
            sfb_t<irq6src_t, 2, 1> PWM2_INT;
            sfb_t<irq6src_t, 3, 1> PWM3_INT;
        };

        union irq7src_t {                           // Interrupt Source Identity Register (IRQ7_SRC)
            __SFR(irq7src_t, uint32_t, 0)
            sfb_t<irq7src_t, 0, 1> PWM4_INT;
            sfb_t<irq7src_t, 1, 1> PWM5_INT;
        };

        union irq8src_t {                           // Interrupt Source Identity Register (IRQ8_SRC)
            __SFR(irq8src_t, uint32_t, 0)
            sfb_t<irq8src_t, 0, 1> TMR0_INT;
        };

        union irq9src_t {                           // Interrupt Source Identity Register (IRQ9_SRC)
            __SFR(irq9src_t, uint32_t, 0)
            sfb_t<irq9src_t, 0, 1> TMR1_INT;
        };

        union irq10src_t {                          // Interrupt Source Identity Register (IRQ10_SRC)
            __SFR(irq10src_t, uint32_t, 0)
            sfb_t<irq10src_t, 0, 1> TMR2_INT;
        };

        union irq11src_t {                          // Interrupt Source Identity Register (IRQ11_SRC)
            __SFR(irq11src_t, uint32_t, 0)
            sfb_t<irq11src_t, 0, 1> TMR3_INT;
        };

        union irq12src_t {                          // Interrupt Source Identity Register (IRQ12_SRC)
            __SFR(irq12src_t, uint32_t, 0)
            sfb_t<irq12src_t, 0, 1> URT0_INT;
        };

        union irq13src_t {                          // Interrupt Source Identity Register (IRQ13_SRC)
            __SFR(irq13src_t, uint32_t, 0)
            sfb_t<irq13src_t, 0, 1> URT1_INT;
        };

        union irq14src_t {                          // Interrupt Source Identity Register (IRQ14_SRC)
            __SFR(irq14src_t, uint32_t, 0)
            sfb_t<irq14src_t, 0, 1> SPI0_INT;
        };

        union irq15src_t {                          // Interrupt Source Identity Register (IRQ15_SRC)
            __SFR(irq15src_t, uint32_t, 0)
            sfb_t<irq15src_t, 0, 1> SPI1_INT;
        };

        union irq18src_t {                          // Interrupt Source Identity Register (IRQ18_SRC)
            __SFR(irq18src_t, uint32_t, 0)
            sfb_t<irq18src_t, 0, 1> I2C0_INT;
        };

        union irq19src_t {                          // Interrupt Source Identity Register (IRQ19_SRC)
            __SFR(irq19src_t, uint32_t, 0)
            sfb_t<irq19src_t, 0, 1> I2C1_INT;
        };

        union irq23src_t {                          // Interrupt Source Identity Register (IRQ23_SRC)
            __SFR(irq23src_t, uint32_t, 0)
            sfb_t<irq23src_t, 0, 1> USB_INT;
        };

        union irq25src_t {                          // Interrupt Source Identity Register (IRQ25_SRC)
            __SFR(irq25src_t, uint32_t, 0)
            sfb_t<irq25src_t, 0, 1> ACMP_INT;
        };

        union irq26src_t {                          // Interrupt Source Identity Register (IRQ26_SRC)
            __SFR(irq26src_t, uint32_t, 0)
            sfb_t<irq26src_t, 0, 1> PDMA_INT;
        };

        union irq27src_t {                          // Interrupt Source Identity Register (IRQ27_SRC)
            __SFR(irq27src_t, uint32_t, 0)
            sfb_t<irq27src_t, 0, 1> I2S_INT;
        };

        union irq28src_t {                          // Interrupt Source Identity Register (IRQ28_SRC)
            __SFR(irq28src_t, uint32_t, 0)
            sfb_t<irq28src_t, 0, 1> PWRWU_INT;
        };

        union irq29src_t {                          // Interrupt Source Identity Register (IRQ29_SRC)
            __SFR(irq29src_t, uint32_t, 0)
            sfb_t<irq29src_t, 0, 1> ADC_INT;
        };

        union irq31src_t {                          // Interrupt Source Identity Register (IRQ31_SRC)
            __SFR(irq31src_t, uint32_t, 0)
            sfb_t<irq31src_t, 0, 1> RTC_INT;
        };

        union nmisel_t {                            // NMI Interrupt Source Select Control Register (NMI_SEL)
            __SFR(nmisel_t, uint32_t, 0)
            sfb_t<nmisel_t, 0, 5> NMI_SEL;
        };

        struct sfrs_t {
            const sfr_t<irq0src_t> IRQ0_SRC;        // Interrupt Source Identity Register (IRQ0_SRC)
            const sfr_t<irq1src_t> IRQ1_SRC;        // Interrupt Source Identity Register (IRQ1_SRC)
            const sfr_t<irq2src_t> IRQ2_SRC;        // Interrupt Source Identity Register (IRQ2_SRC)
            const sfr_t<irq3src_t> IRQ3_SRC;        // Interrupt Source Identity Register (IRQ3_SRC)
            const sfr_t<irq4src_t> IRQ4_SRC;        // Interrupt Source Identity Register (IRQ4_SRC)
            const sfr_t<irq5src_t> IRQ5_SRC;        // Interrupt Source Identity Register (IRQ5_SRC)
            const sfr_t<irq6src_t> IRQ6_SRC;        // Interrupt Source Identity Register (IRQ6_SRC)
            const sfr_t<irq7src_t> IRQ7_SRC;        // Interrupt Source Identity Register (IRQ7_SRC)
            const sfr_t<irq8src_t> IRQ8_SRC;        // Interrupt Source Identity Register (IRQ8_SRC)
            const sfr_t<irq9src_t> IRQ9_SRC;        // Interrupt Source Identity Register (IRQ9_SRC)
            const sfr_t<irq10src_t> IRQ10_SRC;      // Interrupt Source Identity Register (IRQ10_SRC)
            const sfr_t<irq11src_t> IRQ11_SRC;      // Interrupt Source Identity Register (IRQ11_SRC)
            const sfr_t<irq12src_t> IRQ12_SRC;      // Interrupt Source Identity Register (IRQ12_SRC)
            const sfr_t<irq13src_t> IRQ13_SRC;      // Interrupt Source Identity Register (IRQ13_SRC)
            const sfr_t<irq14src_t> IRQ14_SRC;      // Interrupt Source Identity Register (IRQ14_SRC)
            const sfr_t<irq15src_t> IRQ15_SRC;      // Interrupt Source Identity Register (IRQ15_SRC)
            uint64_t :64;
            const sfr_t<irq18src_t> IRQ18_SRC;      // Interrupt Source Identity Register (IRQ18_SRC)
            const sfr_t<irq19src_t> IRQ19_SRC;      // Interrupt Source Identity Register (IRQ19_SRC)
            uint64_t :64;
            uint32_t :32;
            const sfr_t<irq23src_t> IRQ23_SRC;      // Interrupt Source Identity Register (IRQ23_SRC)
            uint32_t :32;
            const sfr_t<irq25src_t> IRQ25_SRC;      // Interrupt Source Identity Register (IRQ25_SRC)
            const sfr_t<irq26src_t> IRQ26_SRC;      // Interrupt Source Identity Register (IRQ26_SRC)
            const sfr_t<irq27src_t> IRQ27_SRC;      // Interrupt Source Identity Register (IRQ27_SRC)
            const sfr_t<irq28src_t> IRQ28_SRC;      // Interrupt Source Identity Register (IRQ28_SRC)
            const sfr_t<irq29src_t> IRQ29_SRC;      // Interrupt Source Identity Register (IRQ29_SRC)
            uint32_t :32;
            const sfr_t<irq31src_t> IRQ31_SRC;      // Interrupt Source Identity Register (IRQ31_SRC)
            const sfr_t<nmisel_t> NMI_SEL;          // NMI Interrupt Source Select Control Register (NMI_SEL)
            volatile uint32_t MCU_IRQ;              // MCU Interrupt Request Source Register (MCU_IRQ)
        };

        extern sfrs_t SYSINT;   SFR_ADDR(SYSINT, SYSINT_ADDR);
    }
}

#endif  // __SFR_SYSINT_HPP
