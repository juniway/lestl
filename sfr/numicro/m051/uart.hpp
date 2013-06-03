#ifndef __SFR_UART_HPP
#define __SFR_UART_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace uart {                                // UART Control Registers
        enum {
            UART0_IRQn = 12,
            UART1_IRQn = 13
        };

        enum {
            UART0_ADDR = APB1_BASE + 0x50000,
            UART1_ADDR = APB2_BASE + 0x50000
        };

        union ier_t {                               // Interrupt Enable Register (UA_IER)
            __SFR(ier_t, uint32_t, 0)
            sfb_t<ier_t, 0, 1> RDA_IEN;
            sfb_t<ier_t, 1, 1> THRE_IEN;
            sfb_t<ier_t, 2, 1> RLS_IEN;
            sfb_t<ier_t, 3, 1> MODEM_IEN;
            sfb_t<ier_t, 4, 1> RTO_IEN;
            sfb_t<ier_t, 5, 1> BUF_ERR_IEN;
            sfb_t<ier_t, 6, 1> WAKE_IEN;
            sfb_t<ier_t, 8, 1> LIN_RX_BRK_IEN;
            sfb_t<ier_t, 11, 1> TIME_OUT_EN;
            sfb_t<ier_t, 12, 1> AUTO_RTS_EN;
            sfb_t<ier_t, 13, 1> AUTO_CTS_EN;
        };

        union fcr_t {                               // FIFO Control Register (UA_FCR)
            __SFR(fcr_t, uint32_t, 0)
            sfb_t<fcr_t, 1, 1> RFR;
            sfb_t<fcr_t, 2, 1> TFR;
            sfb_t<fcr_t, 4, 4> RFITL;
            sfb_t<fcr_t, 8, 1> RX_DIS;
            sfb_t<fcr_t, 16, 4> RTS_TRI_LEVEL;
        };

        union lcr_t {                               // Line Control Register (UA_LCR)
            __SFR(lcr_t, uint32_t, 0)
            sfb_t<lcr_t, 0, 2> WLS;
            sfb_t<lcr_t, 2, 1> NSB;
            sfb_t<lcr_t, 3, 1> PBE;
            sfb_t<lcr_t, 4, 1> EPE;
            sfb_t<lcr_t, 5, 1> SPE;
            sfb_t<lcr_t, 6, 1> BCB;
        };

        union mcr_t {                               // MODEM Control Register (UA_MCR)
            __SFR(mcr_t, uint32_t, 0)
            sfb_t<mcr_t, 1, 1> RTS;
            sfb_t<mcr_t, 9, 1> LEV_RTS;
            sfb_t<mcr_t, 13, 1> RTS_ST;
        };

        union msr_t {                               // Modem Status Register (UA_MSR)
            __SFR(msr_t, uint32_t, 0)
            sfb_t<msr_t, 0, 1> DCTSF;
            sfb_t<msr_t, 4, 1> CTS_ST;
            sfb_t<msr_t, 8, 1> LEV_CTS;
        };

        union fsr_t {                               // FIFO Status Register (UA_FSR)
            __SFR(fsr_t, uint32_t, 0)
            sfb_t<fsr_t, 0, 1> RX_OVER_IF;
            sfb_t<fsr_t, 3, 1> RS485_ADD_DETF;
            sfb_t<fsr_t, 4, 1> PEF;
            sfb_t<fsr_t, 5, 1> FEF;
            sfb_t<fsr_t, 6, 1> BIF;
            sfb_t<fsr_t, 8, 6> RX_POINTER;
            sfb_t<fsr_t, 14, 1> RX_EMPTY;
            sfb_t<fsr_t, 15, 1> RX_FULL;
            sfb_t<fsr_t, 16, 6> TX_POINTER;
            sfb_t<fsr_t, 22, 1> TX_EMPTY;
            sfb_t<fsr_t, 23, 1> TX_FULL;
            sfb_t<fsr_t, 24, 1> TX_OVER_IF;
            sfb_t<fsr_t, 28, 1> TE_FLAG;
        };

        union isr_t {                               // Interrupt Status Control Register (UA_ISR)
            __SFR(isr_t, uint32_t, 0)
            sfb_t<isr_t, 0, 1> RDA_IF;
            sfb_t<isr_t, 1, 1> THRE_IF;
            sfb_t<isr_t, 2, 1> RLS_IF;
            sfb_t<isr_t, 3, 1> MODEM_IF;
            sfb_t<isr_t, 4, 1> TOUT_IF;
            sfb_t<isr_t, 5, 1> BUF_ERR_IF;
            sfb_t<isr_t, 7, 1> LIN_RX_BREAK_IF;
            sfb_t<isr_t, 8, 1> RDA_INT;
            sfb_t<isr_t, 9, 1> THRE_INT;
            sfb_t<isr_t, 10, 1> RLS_INT;
            sfb_t<isr_t, 11, 1> MODEM_INT;
            sfb_t<isr_t, 12, 1> TOUT_INT;
            sfb_t<isr_t, 13, 1> BUF_ERR_INT;
            sfb_t<isr_t, 15, 1> LIN_RX_BREAK_INT;
        };

        union tor_t {                               // Time out Register (UA_TOR)
            __SFR(tor_t, uint32_t, 0)
            sfb_t<tor_t, 0, 8> TOIC;
            sfb_t<tor_t, 8, 8> DLY;
        };

        union baud_t {                              // Baud Rate Divider Register (UA_BAUD)
            __SFR(baud_t, uint32_t, 0)
            sfb_t<baud_t, 0, 16> BRD;
            sfb_t<baud_t, 24, 4> DIVIDER_X;
            sfb_t<baud_t, 28, 1> DIV_X_ONE;
            sfb_t<baud_t, 29, 1> DIV_X_EN;
        };

        union ircr_t {                              // IrDA Control Register (IRCR)
            __SFR(ircr_t, uint32_t, 0)
            sfb_t<ircr_t, 1, 1> TX_SELECT;
            sfb_t<ircr_t, 5, 1> INV_TX;
            sfb_t<ircr_t, 6, 1> INV_RX;
        };

        union alt_csr_t {                           // UART Alternate Control/Status Register (UA_ALT_CSR)
            __SFR(alt_csr_t, uint32_t, 0)
            sfb_t<alt_csr_t, 0, 4> UA_LIN_BKFL;
            sfb_t<alt_csr_t, 6, 1> LIN_RX_EN;
            sfb_t<alt_csr_t, 7, 1> LIN_TX_EN;
            sfb_t<alt_csr_t, 8, 1> RS485_NMM;
            sfb_t<alt_csr_t, 9, 1> RS485_AAD;
            sfb_t<alt_csr_t, 10, 1> RS485_AUD;
            sfb_t<alt_csr_t, 15, 1> RS485_ADD_EN;
            sfb_t<alt_csr_t, 24, 8> ADDR_MATCH;
        };

        union funsel_t {                            // UART Function Select Register (UA_FUN_SEL)
            __SFR(funsel_t, uint32_t, 0)
            sfb_t<funsel_t, 0, 2> FUN_SEL;
        };

        struct uart_t {
            union {
                volatile uint32_t DATA;
                const volatile uint32_t RBR;        // Receive Buffer Register (UA_RBR)
                volatile uint32_t THR;              // Transmit Holding Register (UA_THR)
            };
            volatile sfr_t<ier_t> IER;              // Interrupt Enable Register (UA_IER)
            volatile sfr_t<fcr_t> FCR;              // FIFO Control Register (UA_FCR)
            volatile sfr_t<lcr_t> LCR;              // Line Control Register (UA_LCR)
            volatile sfr_t<mcr_t> MCR;              // MODEM Control Register (UA_MCR)
            volatile sfr_t<msr_t> MSR;              // Modem Status Register (UA_MSR)
            volatile sfr_t<fsr_t> FSR;              // FIFO Status Register (UA_FSR)
            volatile sfr_t<isr_t> ISR;              // Interrupt Status Control Register (UA_ISR)
            volatile sfr_t<tor_t> TOR;              // Time out Register (UA_TOR)
            volatile sfr_t<baud_t> BAUD;            // Baud Rate Divider Register (UA_BAUD)
            volatile sfr_t<ircr_t> IRCR;            // IrDA Control Register (IRCR)
            volatile sfr_t<alt_csr_t> ALT_CSR;      // UART Alternate Control/Status Register (UA_ALT_CSR)
            volatile sfr_t<funsel_t> FUN_SEL;       // UART Function Select Register (UA_FUN_SEL)
        };

        extern uart_t UART0;    SFR_ADDR(UART0, UART0_ADDR);
        extern uart_t UART1;    SFR_ADDR(UART1, UART1_ADDR);
    }
}

#endif  // __SFR_UART_HPP
