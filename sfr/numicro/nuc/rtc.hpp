#ifndef __SFR_RTC_HPP
#define __SFR_RTC_HPP

#include <sfr/sfr.hpp>
#include <sfr/numicro/bus_base.h>

namespace sfr {
    namespace rtc {                                 // Real Time Clock(RTC) Control Registers
        enum {
            RTC_IRQn = 31
        };

        enum {
            RTC_ADDR = APB1_BASE + 0x8000
        };

        union inir_t {                              // RTC Initiation Register (INIR)
            __SFR(inir_t, uint32_t, 0)
            sfb_t<inir_t, 0, 1> ACTIVE;
            sfb_t<inir_t, 0, 32> INIR;
        };

        union aer_t {                               // RTC Access Enable Register (AER)
            __SFR(aer_t, uint32_t, 0)
            sfb_t<aer_t, 0, 16> AER;
            sfb_t<aer_t, 16, 1> ENF;
        };

        union fcr_t {                               // RTC Frequency Compensation Register (FCR)
            __SFR(fcr_t, uint32_t, 0)
            sfb_t<fcr_t, 0, 6> FRACTION;
            sfb_t<fcr_t, 8, 4> INTEGER;
        };

        union tr_t {                                // RTC Time Register
            __SFR(tr_t, uint32_t, 0)
            sfb_t<tr_t, 0, 4> SEC1;
            sfb_t<tr_t, 4, 3> SEC10;
            sfb_t<tr_t, 8, 4> MIN1;
            sfb_t<tr_t, 12, 3> MIN10;
            sfb_t<tr_t, 16, 4> HR1;
            sfb_t<tr_t, 20, 2> HR10;
            sfb_t<tr_t, 0, 7> SEC;
            sfb_t<tr_t, 8, 7> MIN;
            sfb_t<tr_t, 16, 6> HOUR;
        };

        union cr_t {                                // RTC Calendar Register
            __SFR(cr_t, uint32_t, 0)
            sfb_t<cr_t, 0, 4> DAY1;
            sfb_t<cr_t, 4, 2> DAY10;
            sfb_t<cr_t, 8, 4> MON1;
            sfb_t<cr_t, 12, 1> MON10;
            sfb_t<cr_t, 16, 4> YEAR1;
            sfb_t<cr_t, 20, 4> YEAR10;
            sfb_t<cr_t, 0, 6> DAY;
            sfb_t<cr_t, 8, 5> MON;
            sfb_t<cr_t, 16, 8> YEAR;
        };

        union tssr_t {                              // RTC Time Scale Selection Register (TSSR)
            __SFR(tssr_t, uint32_t, 0)
            sfb_t<tssr_t, 0, 1> HR24;
        };

        union dwr_t {                               // RTC Day of the Week Register (DWR)
            __SFR(dwr_t, uint32_t, 0)
            sfb_t<dwr_t, 0, 3> DWR;
        };

        union lir_t {                               // RTC Leap year Indication Register (LIR)
            __SFR(lir_t, uint32_t, 0)
            sfb_t<lir_t, 0, 1> LIR;
        };

        union rier_t {                              // RTC Interrupt Enable Register (RIER)
            __SFR(rier_t, uint32_t, 0)
            sfb_t<rier_t, 0, 1> AIER;
            sfb_t<rier_t, 1, 1> TIER;
        };

        union riir_t {                              // RTC Interrupt Indication Register (RIIR)
            __SFR(riir_t, uint32_t, 0)
            sfb_t<riir_t, 0, 1> AI;
            sfb_t<riir_t, 1, 1> TI;
        };

        union ttr_t {                               // RTC Time Tick Register (TTR)
            __SFR(ttr_t, uint32_t, 0)
            sfb_t<ttr_t, 0, 3> TTR;
            sfb_t<ttr_t, 3, 1> TWKE;
        };

        struct rtc_t {
            volatile sfr_t<inir_t> INIR;            // RTC Initiation Register (INIR)
            volatile sfr_t<aer_t> AER;              // RTC Access Enable Register (AER)
            volatile sfr_t<fcr_t> FCR;              // RTC Frequency Compensation Register (FCR)
            volatile sfr_t<tr_t> TLR;               // RTC Time Loading Register (TLR)
            volatile sfr_t<cr_t> CLR;               // RTC Calendar Loading Register (CLR)
            volatile sfr_t<tssr_t> TSSR;            // RTC Time Scale Selection Register (TSSR)
            volatile sfr_t<dwr_t> DWR;              // RTC Day of the Week Register (DWR)
            volatile sfr_t<tr_t> TAR;               // RTC Time Alarm Register (TAR)
            volatile sfr_t<cr_t> CAR;               // RTC Calendar Alarm Register (CAR)
            const volatile sfr_t<lir_t> LIR;        // RTC Leap year Indication Register (LIR)
            volatile sfr_t<rier_t> RIER;            // RTC Interrupt Enable Register (RIER)
            volatile sfr_t<riir_t> RIIR;            // RTC Interrupt Indication Register (RIIR)
            volatile sfr_t<ttr_t> TTR;              // RTC Time Tick Register (TTR)
        };

        extern rtc_t RTC;   SFR_ADDR(RTC, RTC_ADDR);
    }
}

#endif  // __SFR_RTC_HPP
