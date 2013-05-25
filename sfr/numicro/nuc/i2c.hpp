#ifndef __SFR_I2C_HPP
#define __SFR_I2C_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace i2c {
        enum {
            I2C0_IRQn = 18,
            I2C1_IRQn = 19
        };

        enum {
            I2C0_ADDR = APB1_BASE + 0x20000,
            I2C1_ADDR = APB2_BASE + 0x20000
        };

        enum status_t {
            I2C_START                   = 0x08,     // start condition transmitted
            I2C_REP_START               = 0x10,     // repeated start condition transmitted
            // Master Transmitter
            I2C_MT_SLA_ACK              = 0x18,     // SLA+W transmitted, ACK received
            I2C_MT_SLA_NACK             = 0x20,     // SLA+W transmitted, NACK received
            I2C_MT_DATA_ACK             = 0x28,     // data transmitted, ACK received
            I2C_MT_DATA_NACK            = 0x30,     // data transmitted, NACK received
            I2C_MT_ARB_LOST             = 0x38,     // arbitration lost in SLA+W or data
            // Master Receiver
            I2C_MR_ARB_LOST             = 0x38,     // arbitration lost in SLA+R or NACK
            I2C_MR_SLA_ACK              = 0x40,     // SLA+R transmitted, ACK received
            I2C_MR_SLA_NACK             = 0x48,     // SLA+R transmitted, NACK received
            I2C_MR_DATA_ACK             = 0x50,     // data received, ACK returned
            I2C_MR_DATA_NACK            = 0x58,     // data received, NACK returned
            // Slave Transmitter
            I2C_ST_SLA_ACK              = 0xA8,     // SLA+R received, ACK returned
            I2C_ST_ARB_LOST_SLA_ACK     = 0xB0,     // arbitration lost in SLA+RW, SLA+R received, ACK returned
            I2C_ST_DATA_ACK             = 0xB8,     // data transmitted, ACK received
            I2C_ST_DATA_NACK            = 0xC0,     // data transmitted, NACK received
            I2C_ST_LAST_DATA            = 0xC8,     // last data byte transmitted, ACK received
            // Slave Receiver
            I2C_SR_SLA_ACK              = 0x60,     // SLA+W received, ACK returned
            I2C_SR_ARB_LOST_SLA_ACK     = 0x68,     // arbitration lost in SLA+RW, SLA+W received, ACK returned
            I2C_SR_GCALL_ACK            = 0x70,     // general call received, ACK returned
            I2C_SR_ARB_LOST_GCALL_ACK   = 0x78,     // arbitration lost in SLA+RW, general call received, ACK returned
            I2C_SR_DATA_ACK             = 0x80,     // data received, ACK returned
            I2C_SR_DATA_NACK            = 0x88,     // data received, NACK returned
            I2C_SR_GCALL_DATA_ACK       = 0x90,     // general call data received, ACK returned
            I2C_SR_GCALL_DATA_NACK      = 0x98,     // general call data received, NACK returned
            I2C_SR_STOP                 = 0xA0,     // stop or repeated start condition received while selected
            // Misc
            I2C_NO_INFO                 = 0xF8,     // no state information available
            I2C_BUS_ERROR               = 0x00      // illegal start or stop condition
        };

        union con_t {                               // I2C Control Register (I2CON)
            __SFR(con_t, uint32_t, 0)
            sfb_t<con_t, 2, 1> AA;
            sfb_t<con_t, 3, 1> SI;
            sfb_t<con_t, 4, 1> STO;
            sfb_t<con_t, 5, 1> STA;
            sfb_t<con_t, 6, 1> ENS1;
            sfb_t<con_t, 7, 1> EI;
        };

        union addr_t {                              // I2C Slave Address Register (I2CADDR)
            __SFR(addr_t, uint32_t, 0)
            sfb_t<addr_t, 0, 1> GC;
            sfb_t<addr_t, 1, 7> ADDR;
        };

        union toc_t {                               // I2C Time-Out Counter Register (I2CTOC)
            __SFR(toc_t, uint32_t, 0)
            sfb_t<toc_t, 0, 1> TIF;
            sfb_t<toc_t, 1, 1> DIV4;
            sfb_t<toc_t, 2, 1> ENTI;
        };

        union adrm_t {                              // I2C Slave Address Mask Register (I2CADM)
            __SFR(adrm_t, uint32_t, 0)
            sfb_t<adrm_t, 1, 7> ADM;
        };

        struct i2c_t {                              // I2C Interface Control Registers
            volatile sfr_t<con_t> CON;              // I2C Control Register (I2CON)
            volatile sfr_t<addr_t> ADDR0;           // I2C Slave Address Register (I2CADDR0)
            volatile uint32_t DATA;                 // I2C Data Register (I2CDAT)
            const volatile status_t STATUS;         // I2C Status Register (I2CSTATUS)
            volatile uint32_t CLK;                  // I2C Clock Divided Register (I2CLK)
            volatile sfr_t<toc_t> TOC;              // I2C Time-Out Counter Register (I2CTOC)
            volatile sfr_t<addr_t> ADDR1;           // I2C Slave Address Register (I2CADDR1)
            volatile sfr_t<addr_t> ADDR2;           // I2C Slave Address Register (I2CADDR2)
            volatile sfr_t<addr_t> ADDR3;           // I2C Slave Address Register (I2CADDR3)
            volatile sfr_t<adrm_t> ADRM0;           // I2C Slave Address Mask Register (I2CADM0)
            volatile sfr_t<adrm_t> ADRM1;           // I2C Slave Address Mask Register (I2CADM1)
            volatile sfr_t<adrm_t> ADRM2;           // I2C Slave Address Mask Register (I2CADM2)
            volatile sfr_t<adrm_t> ADRM3;           // I2C Slave Address Mask Register (I2CADM3)
        };

        extern i2c_t I2C0;  SFR_ADDR(I2C0, I2C0_ADDR);
        extern i2c_t I2C1;  SFR_ADDR(I2C1, I2C1_ADDR);
    }
}

#endif  // __SFR_I2C_HPP
