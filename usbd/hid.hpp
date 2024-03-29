/*
  Copyright (c) 2012-2013  John Lee (j.y.lee@yeah.net)
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.

  * Neither the name of the copyright holders nor the names of
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __USBD_HID_HPP
#define __USBD_HID_HPP

#include <usbd/core>

namespace usbd {
    namespace hid {
        enum class desc_type_t : uint8_t {
            HID = 0x21,
            REPORT,
            PHYSICAL
        };

        namespace helper {
            template<typename DESC>
            struct __attribute__((packed)) elem_t {
                uint8_t bReportDescriptorType{ uint8_t(DESC::type) };
                uint16_t __attribute__((packed)) wReportDescriptorLength{ sizeof(DESC) };
            };
    
            template<typename DESC, typename... DESCs>
            struct __attribute__((packed)) optional_t : elem_t<DESC>, optional_t<DESCs...> { };
    
            template<typename DESC>
            struct __attribute__((packed)) optional_t<DESC> : elem_t<DESC> { };
        }

        template<typename DESC, typename... DESCs>
        struct __attribute__((packed)) optional_t {
            using report_t = DESC;
            uint8_t bNumDescriptors{ 1 + sizeof...(DESCs) };
            helper::optional_t<DESC, DESCs...> optional;
        };

        template<uint8_t SIZE, uint8_t TYPE, uint8_t TAG>
        struct __attribute__((packed)) item_prefix_t {
            uint8_t prefix{ SIZE | (TYPE << 2) | (TAG << 4) };
        };

        template<uint8_t TYPE, uint8_t TAG, uint8_t... DATAs> struct __attribute__((packed)) short_item_t;

        template<uint8_t TYPE, uint8_t TAG>
        struct __attribute__((packed)) short_item_t<TYPE, TAG> : item_prefix_t<0, TYPE, TAG> { };

        template<uint8_t TYPE, uint8_t TAG, uint8_t DATA0>
        struct __attribute__((packed)) short_item_t<TYPE, TAG, DATA0> : item_prefix_t<1, TYPE, TAG> {
            uint8_t data{ DATA0 };
        };

        template<uint8_t TYPE, uint8_t TAG, uint8_t DATA0, uint8_t DATA1>
        struct __attribute__((packed)) short_item_t<TYPE, TAG, DATA0, DATA1> : item_prefix_t<2, TYPE, TAG> {
            uint16_t __attribute__((packed)) data{ DATA0 | (DATA1 << 8) };
        };

        template<uint8_t TYPE, uint8_t TAG, uint8_t DATA0, uint8_t DATA1, uint8_t DATA2, uint8_t DATA3>
        struct __attribute__((packed)) short_item_t<TYPE, TAG, DATA0, DATA1, DATA2, DATA3> : item_prefix_t<3, TYPE, TAG> {
            uint32_t __attribute__((packed)) data{ DATA0 | (DATA1 << 8) | (DATA2 << 16) | (DATA3 << 24) };
        };

        template<uint8_t TYPE, uint8_t TAG, int64_t DATA> struct __attribute__((packed)) short_item_signed_t : std::conditional<
                DATA >= -128 && DATA <= 127,
                short_item_t<TYPE, TAG, DATA>,
                typename std::conditional<
                    DATA >= -32768 && DATA <= 32767,
                    short_item_t<TYPE, TAG, DATA & 0xff, (DATA >> 8) & 0xff>,
                    short_item_t<TYPE, TAG, DATA & 0xff, (DATA >> 8) & 0xff, (DATA >> 16) & 0xff, (DATA >> 24) & 0xff>
                >::type
            >::type { };

        template<uint8_t TYPE, uint8_t TAG, uint64_t DATA> struct __attribute__((packed)) short_item_unsigned_t : std::conditional<
                DATA < 256,
                short_item_t<TYPE, TAG, DATA>,
                typename std::conditional<
                    DATA < 65536,
                    short_item_t<TYPE, TAG, DATA & 0xff, (DATA >> 8) & 0xff>,
                    short_item_t<TYPE, TAG, DATA & 0xff, (DATA >> 8) & 0xff, (DATA >> 16) & 0xff, (DATA >> 24) & 0xff>
                >::type
            >::type { };

        template<uint8_t TYPE, uint8_t TAG, uint8_t... DATAs>
        struct __attribute__((packed)) long_item_t : public item_prefix_t<2, 3, 0xf> {
            uint8_t data_size{ sizeof...(DATAs) };
            uint8_t item_tag{ TAG };
            uint8_t data[sizeof...(DATAs)]{ DATAs... };
        };

        template<uint8_t TYPE, uint8_t TAG>
        struct __attribute__((packed)) long_item_t<TYPE, TAG> : public item_prefix_t<2, 3, 0xf> {
            uint8_t data_size{ 0 };
            uint8_t item_tag{ TAG };
        };

        template<uint8_t N, typename ITEM>
        struct __attribute__((packed)) report_item_t { ITEM item; };

        template<uint8_t N, typename ITEM, typename... ITEMs>
        struct __attribute__((packed)) report_tuple_t : report_item_t<N, ITEM>, report_tuple_t<N + 1, ITEMs...> { };

        template<uint8_t N, typename ITEM>
        struct __attribute__((packed)) report_tuple_t<N, ITEM> : report_item_t<N, ITEM> { };

        // main
        enum iof_value_t : uint64_t {
            DATA                = 0x00100000000ULL,
            CONSTANT            = 0x00100000001ULL,
            ARRAY               = 0x00200000000ULL,
            VARIABLE            = 0x00200000002ULL,
            ABSOLUTE            = 0x00400000000ULL,
            RELATIVE            = 0x00400000004ULL,
            NO_WRAP             = 0x00800000000ULL,
            WARP                = 0x00800000008ULL,
            LINEAR              = 0x01000000000ULL,
            NO_LINEAR           = 0x01000000010ULL,
            PREFERRED_STATE     = 0x02000000000ULL,
            NO_PREFERRED        = 0x02000000020ULL,
            NO_NULL_POSITION    = 0x04000000000ULL,
            NULL_STATE          = 0x04000000040ULL,
            NO_VOLATILE         = 0x08000000000ULL,
            VOLATILE            = 0x08000000080ULL,
            BIT_FIELD           = 0x10000000000ULL,
            BUFFERED            = 0x10000000100ULL
        };

        template<iof_value_t DATA, iof_value_t... DATAs> struct iof_helper_t : iof_helper_t<DATAs...> {
            static_assert(((uint64_t(DATA) & iof_helper_t<DATAs...>::value) & ~0xffffffffULL) == 0, "...");
            static const uint64_t value{ uint64_t(DATA) | iof_helper_t<DATAs...>::value };
        };

        template<iof_value_t DATA> struct iof_helper_t<DATA> {
            static const uint64_t value{ uint64_t(DATA) };
        };

        enum collection_value_t {
            PHYSICAL = 0,
            APPLICATION = 1,
            LOGICAL = 2,
            REPORT = 3,
            NAMED_ARRAY = 4,
            USAGE_SWITCH = 5,
            USAGE_MODIFIER = 6
        };

        template<iof_value_t... DATAs> struct __attribute__((packed)) input_t : short_item_unsigned_t<0, 8, iof_helper_t<DATAs...>::value & 0xffffffff> { };
        template<iof_value_t... DATAs> struct __attribute__((packed)) output_t : short_item_unsigned_t<0, 9, iof_helper_t<DATAs...>::value & 0xffffffff> { };
        template<collection_value_t DATA, typename ITEM, typename... ITEMs>
        struct __attribute__((packed)) collection_t : short_item_t<0, 10, uint8_t(DATA)>, report_tuple_t<0, ITEM, ITEMs...>, short_item_t<0, 12> { };
        template<iof_value_t... DATAs> struct __attribute__((packed)) feature_t : short_item_t<0, 11, iof_helper_t<DATAs...>::value & 0xffffffff> { };
        // global
        enum usage_page_value_t {
            GENERIC_DESKTOP = 1,
            SIMULATION_CONTROLS = 2,
            VR_CONTROLS = 3,
            SPORT_CONTROLS = 4,
            GAME_CONTROLS = 5,
            GENERIC_DEVICE = 6,
            KEYBOARD = 7,
            LEDS = 8,
            BUTTON = 9,
            ORDINALS = 10,
            TELEPHONY_DEVICE = 11,
            CONSUMER_DEVICE = 12,
            DIGITIZER = 13,
            UNICODE = 16,
            ALPHANUMERIC_DISPLAY = 20,
            MEDICAL_INSTRUMENTS = 64
        };
        template<uint16_t DATA> struct __attribute__((packed)) usage_page_t : short_item_unsigned_t<1, 0, DATA> { };
        template<int64_t DATA> struct __attribute__((packed)) logical_minimum_t : short_item_signed_t<1, 1, DATA> { };
        template<int64_t DATA> struct __attribute__((packed)) logical_maximum_t : short_item_signed_t<1, 2, DATA> { };
        template<int64_t DATA> struct __attribute__((packed)) physical_minimum_t : short_item_signed_t<1, 3, DATA> { };
        template<int64_t DATA> struct __attribute__((packed)) physical_maximum_t : short_item_signed_t<1, 4, DATA> { };
        template<int8_t DATA> struct __attribute__((packed)) unit_exponent_t : short_item_t<1, 5, DATA & 0xf> { };
        template<uint8_t... DATAs> struct __attribute__((packed)) unit_t : short_item_t<1, 6, DATAs...> { };
        template<uint64_t DATA> struct __attribute__((packed)) report_size_t : short_item_unsigned_t<1, 7, DATA> { };
        template<uint8_t DATA> struct __attribute__((packed)) report_id_t : short_item_t<1, 8, DATA> { };
        template<uint64_t DATA> struct __attribute__((packed)) report_count_t : short_item_unsigned_t<1, 9, DATA> { };
        struct __attribute__((packed)) push_t : short_item_t<1, 10> { };
        struct __attribute__((packed)) pop_t : short_item_t<1, 11> { };

        template<int64_t MIN, int64_t MAX> struct __attribute__((packed)) logical_extremum_t : logical_minimum_t<MIN>, logical_maximum_t<MAX> { };
        template<int64_t MIN, int64_t MAX> struct __attribute__((packed)) physical_extremum_t : physical_minimum_t<MIN>, physical_maximum_t<MAX> { };
        // local
        enum delimiter_value_t {
            CLOSE = 0,
            OPEN = 1
        };
        template<uint8_t... DATAs> struct __attribute__((packed)) usage_t : short_item_t<2, 0, DATAs...> { };
        template<int64_t DATA> struct __attribute__((packed)) usage_minimum_t : short_item_signed_t<2, 1, DATA> { };
        template<int64_t DATA> struct __attribute__((packed)) usage_maximum_t : short_item_signed_t<2, 2, DATA> { };
        template<uint8_t DATA> struct __attribute__((packed)) designator_index_t : short_item_t<2, 3, DATA> { };
        template<uint64_t DATA> struct __attribute__((packed)) designator_minimum_t : short_item_unsigned_t<2, 4, DATA> { };
        template<uint64_t DATA> struct __attribute__((packed)) designator_maximum_t : short_item_unsigned_t<2, 5, DATA> { };
        template<uint8_t DATA> struct __attribute__((packed)) string_index_t : short_item_t<2, 7, DATA> { };
        template<uint64_t DATA> struct __attribute__((packed)) string_minimum_t : short_item_unsigned_t<2, 8, DATA> { };
        template<uint64_t DATA> struct __attribute__((packed)) string_maximum_t : short_item_unsigned_t<2, 9, DATA> { };
        template<delimiter_value_t DATA> struct __attribute__((packed)) delimiter_t : short_item_t<2, 10, uint8_t(DATA)> { };

        template<int64_t MIN, int64_t MAX> struct __attribute__((packed)) usage_extremum_t : usage_minimum_t<MIN>, usage_maximum_t<MAX> { };
        template<uint64_t MIN, uint64_t MAX> struct __attribute__((packed)) designator_extremum_t : designator_minimum_t<MIN>, designator_maximum_t<MAX> { };
        template<uint64_t MIN, uint64_t MAX> struct __attribute__((packed)) string_extremum_t : string_minimum_t<MIN>, string_maximum_t<MAX> { };

        template<typename... ITEMs>
        struct __attribute__((packed)) report_t : report_tuple_t<0, ITEMs...> {
            static const desc_type_t type{ desc_type_t::REPORT };
        };

        enum class request_t {
            GET_REPORT = 1,
            GET_IDLE,
            GET_PROTOCOL,
            SET_REPORT = 9,
            SET_IDLE,
            SET_PROTOCOL
        };

        namespace in {
            template<template<typename, uint32_t> class EP, typename PARENT, uint32_t PARAM, uint8_t INTERVAL>
            using ep_impl_t = core::transfer::interrupt::in::ep_impl_t<EP, PARENT, PARAM, INTERVAL>;
        };

        namespace out {
            template<template<typename, uint32_t> class EP, typename PARENT, uint32_t PARAM, uint8_t INTERVAL>
            using ep_impl_t = core::transfer::interrupt::out::ep_impl_t<EP, PARENT, PARAM, INTERVAL>;
        };

        namespace base {
            template<template<typename, uint32_t> class HID, typename PARENT, uint32_t PARAM, uint8_t SCLS, uint8_t PROTO, uint8_t STR, uint16_t VER, uint8_t COUNTRY, typename DESC, template<typename, uint32_t> class... EPs>
            class if_impl_t : public core::if_impl_t<HID, PARENT, PARAM, 3, SCLS, PROTO, STR, EPs...> {
                using typename if_impl_t::super_t::if_t;

            protected:
                __INLINE constexpr if_impl_t() { }

            public:
                using super_t = if_impl_t;
                struct __attribute__((packed)) class_descriptor_t {
                    uint8_t bLength{ sizeof(class_descriptor_t) };
                    desc_type_t bDescriptorType{ desc_type_t::HID };
                    uint16_t __attribute__((packed)) bcdHID{ VER };
                    uint8_t bCountryCode{ COUNTRY };
                    DESC optional;
                };

            public:
                __INLINE bool get_descriptor_setup(uint_fast16_t value, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    assert(length != 0);
                    uint_fast8_t index = value & 0xff;
                    switch (desc_type_t(value >> 8)) {
                    case desc_type_t::HID:
                        assert(index == 0);
                        return hid.get_hid_descriptor_setup(length);
                    case desc_type_t::REPORT:
                        assert(index == 0);
                        return hid.get_report_descriptor_setup(length);
                    case desc_type_t::PHYSICAL:
                        return hid.get_physical_descriptor_setup(index, length);
                    default:
                        assert(false);
                        return false;
                    }
                }
                __INLINE bool get_descriptor_data(uint_fast16_t value, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    assert(length != 0);
                    uint_fast8_t index = value & 0xff;
                    switch (desc_type_t(value >> 8)) {
                    case desc_type_t::HID:
                        return hid.get_hid_descriptor_data(length);
                    case desc_type_t::REPORT:
                        return hid.get_report_descriptor_data(length);
                    case desc_type_t::PHYSICAL:
                        return hid.get_physical_descriptor_data(index, length);
                    default:
                        assert(false);
                        get_usb(hid).in();
                        return true;
                    }
                }
                __INLINE bool get_descriptor_status(uint_fast16_t value, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    assert(length != 0);
                    uint_fast8_t index = value & 0xff;
                    switch (desc_type_t(value >> 8)) {
                    case desc_type_t::HID:
                        return hid.get_hid_descriptor_status(length);
                    case desc_type_t::REPORT:
                        return hid.get_report_descriptor_status(length);
                    case desc_type_t::PHYSICAL:
                        return hid.get_physical_descriptor_status(index, length);
                    default:
                        assert(false);
                        return true;
                    }
                }
                __INLINE bool set_descriptor_setup(uint_fast16_t value, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    assert(length != 0);
                    uint_fast8_t index = value & 0xff;
                    switch (desc_type_t(value >> 8)) {
                    case desc_type_t::HID:
                        assert(index == 0);
                        return hid.set_hid_descriptor_setup(length);
                    case desc_type_t::REPORT:
                        assert(index == 0);
                        return hid.set_report_descriptor_setup(length);
                    case desc_type_t::PHYSICAL:
                        return hid.set_physical_descriptor_setup(index, length);
                    default:
                        assert(false);
                        return false;
                    }
                }
                __INLINE bool set_descriptor_data(uint_fast16_t value, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    assert(length != 0);
                    uint_fast8_t index = value & 0xff;
                    switch (desc_type_t(value >> 8)) {
                    case desc_type_t::HID:
                        return hid.set_hid_descriptor_data(length);
                    case desc_type_t::REPORT:
                        return hid.set_report_descriptor_data(length);
                    case desc_type_t::PHYSICAL:
                        return hid.set_physical_descriptor_data(index, length);
                    default:
                        assert(false);
                        get_usb(hid).out();
                        return true;
                    }
                }
                __INLINE bool set_descriptor_status(uint_fast16_t value, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    assert(length != 0);
                    uint_fast8_t index = value & 0xff;
                    switch (desc_type_t(value >> 8)) {
                    case desc_type_t::HID:
                        return hid.set_hid_descriptor_status(length);
                    case desc_type_t::REPORT:
                        return hid.set_report_descriptor_status(length);
                    case desc_type_t::PHYSICAL:
                        return hid.set_physical_descriptor_status(index, length);
                    default:
                        assert(false);
                        return true;
                    }
                }
                __INLINE bool class_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    switch (request_t(request)) {
                    case request_t::SET_REPORT:
                        return hid.set_report_setup(value >> 8, value & 0xff, length);
                    case request_t::SET_IDLE:
                        assert(length == 0);
                        return hid.set_idle_setup(value >> 8, value & 0xff);
                    case request_t::SET_PROTOCOL:
                        assert(value == 0 || value == 1);
                        assert(length == 0);
                        return hid.set_protocol_setup(value);
                    default:
                        assert(false);
                        return false;
                    }
                }
                __INLINE bool class_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    switch (request_t(request)) {
                    case request_t::GET_REPORT:
                        assert(length != 0);
                        return hid.get_report_setup(value >> 8, value & 0xff, length);
                    case request_t::GET_IDLE:
                        assert(value >> 8 == 0);
                        assert(length == 1);
                        return hid.get_idle_setup(value, length);
                    case request_t::GET_PROTOCOL:
                        assert(value == 0);
                        assert(length == 1);
                        return hid.get_protocol_setup(length);
                    default:
                        assert(false);
                        return false;
                    }
                }
                __INLINE bool class_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    switch (request_t(request)) {
                    case request_t::SET_REPORT:
                        return hid.set_report_data(value >> 8, value & 0xff, length);
                    default:
                        assert(false);
                        get_usb(hid).out();
                        return true;
                    }
                }
                __INLINE bool class_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    switch (request_t(request)) {
                    case request_t::GET_REPORT:
                        return hid.get_report_data(value >> 8, value & 0xff, length);
                    case request_t::GET_IDLE:
                        return hid.get_idle_data(value, length);
                    case request_t::GET_PROTOCOL:
                        return hid.get_protocol_data(value);
                    default:
                        assert(false);
                        get_usb(hid).in();
                        return true;
                    }
                }
                __INLINE bool class_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    switch (request_t(request)) {
                    case request_t::SET_REPORT:
                        return hid.set_report_status(value >> 8, value & 0xff, length);
                    case request_t::SET_IDLE:
                        return hid.set_idle_status(value >> 8, value & 0xff);
                    case request_t::SET_PROTOCOL:
                        return hid.set_protocol_status(value);
                    default:
                        assert(false);
                        return true;
                    }
                }
                __INLINE bool class_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    switch (request_t(request)) {
                    case request_t::GET_REPORT:
                        return hid.get_report_status(value >> 8, value & 0xff, length);
                    case request_t::GET_IDLE:
                        return hid.get_idle_status(value, length);
                    case request_t::GET_PROTOCOL:
                        return hid.get_protocol_status(value);
                    default:
                        assert(false);
                        return true;
                    }
                }

                __INLINE bool get_hid_descriptor_setup(uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    auto& descriptor = hid.get_hid_descriptor(length);
                    if (&descriptor != nullptr) {
                        get_usb(hid).in(&descriptor, sizeof(descriptor), length);
                        return true;
                    }
                    return false;
                }
                __INLINE bool get_report_descriptor_setup(uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    auto& descriptor = hid.get_report_descriptor(length);
                    get_usb(hid).in(&descriptor, sizeof(descriptor), length);
                    return true;
                }
                __INLINE bool get_physical_descriptor_setup(uint_fast8_t index, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    auto& descriptor = hid.get_physical_descriptor(index, length);
                    if (&descriptor != nullptr) {
                        get_usb(hid).in(&descriptor, sizeof(descriptor), length);
                        return true;
                    }
                    return false;
                }
                __INLINE bool set_hid_descriptor_setup(uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    void* buffer = hid.set_hid_descriptor(length);
                    if (buffer != nullptr) {
                        get_usb(hid).out(buffer, length);
                        return true;
                    }
                    return false;
                }
                __INLINE bool set_report_descriptor_setup(uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    void* buffer = hid.set_report_descriptor(length);
                    if (buffer != nullptr) {
                        get_usb(hid).out(buffer, length);
                        return true;
                    }
                    return false;
                }
                __INLINE bool set_physical_descriptor_setup(uint_fast8_t index, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    void* buffer = hid.set_physical_descriptor(index, length);
                    if (buffer != nullptr) {
                        get_usb(hid).out(buffer, length);
                        return true;
                    }
                    return false;
                }
                __INLINE bool get_report_setup(uint_fast8_t type, uint_fast8_t id, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    const void* buffer = hid.get_report(type, id, length);
                    if (buffer != nullptr) {
                        get_usb(hid).in(buffer, length, length);
                        return true;
                    }
                    return false;
                }
                __INLINE bool get_idle_setup(uint_fast8_t id, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    const void* buffer = hid.get_idle(id, length);
                    if (buffer != nullptr) {
                        get_usb(hid).in(buffer, length, length);
                        return true;
                    }
                    return false;
                }
                __INLINE bool get_protocol_setup(uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    const void* buffer = hid.get_protocol(length);
                    if (buffer != nullptr) {
                        get_usb(hid).in(buffer, length, length);
                        return true;
                    }
                    return false;
                }
                __INLINE bool set_report_setup(uint_fast8_t type, uint_fast8_t id, uint_fast16_t length)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    void* buffer = hid.set_report(type, id, length);
                    if (buffer != nullptr) {
                        get_usb(hid).out(buffer, length);
                        return true;
                    }
                    return false;
                }
                __INLINE bool set_idle_setup(uint_fast8_t duration, uint_fast8_t id)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    if (hid.set_idle(duration, id)) {
                        get_usb(hid).out(nullptr, 0);
                        return true;
                    }
                    return false;
                }
                __INLINE bool set_protocol_setup(uint_fast8_t protocol)
                {
                    auto& hid = *static_cast<if_t*>(this);
                    if (hid.set_protocol(protocol)) {
                        get_usb(hid).out(nullptr, 0);
                        return true;
                    }
                    return false;
                }

                __INLINE bool get_hid_descriptor_data(uint_fast16_t length)
                {
                    get_usb(*this).in();
                    return true;
                }
                __INLINE bool get_report_descriptor_data(uint_fast16_t length)
                {
                    get_usb(*this).in();
                    return true;
                }
                __INLINE bool get_physical_descriptor_data(uint_fast8_t index, uint_fast16_t length)
                {
                    get_usb(*this).in();
                    return true;
                }
                __INLINE bool set_hid_descriptor_data(uint_fast16_t length)
                {
                    get_usb(*this).out();
                    return true;
                }
                __INLINE bool set_report_descriptor_data(uint_fast16_t length)
                {
                    get_usb(*this).out();
                    return true;
                }
                __INLINE bool set_physical_descriptor_data(uint_fast8_t index, uint_fast16_t length)
                {
                    get_usb(*this).out();
                    return true;
                }
                __INLINE bool get_report_data(uint_fast8_t type, uint_fast8_t id, uint_fast16_t length)
                {
                    get_usb(*this).in();
                    return true;
                }
                __INLINE bool get_idle_data(uint_fast8_t id, uint_fast16_t length)
                {
                    get_usb(*this).in();
                    return true;
                }
                __INLINE bool get_protocol_data(uint_fast16_t length)
                {
                    get_usb(*this).in();
                    return true;
                }
                __INLINE bool set_report_data(uint_fast8_t type, uint_fast8_t id, uint_fast16_t length)
                {
                    get_usb(*this).out();
                    return true;
                }

                __INLINE bool get_hid_descriptor_status(uint_fast16_t length) { return true; }
                __INLINE bool get_report_descriptor_status(uint_fast16_t length) { return true; }
                __INLINE bool get_physical_descriptor_status(uint_fast8_t index, uint_fast16_t length) { return true; }
                __INLINE bool set_hid_descriptor_status(uint_fast16_t length) { return true; }
                __INLINE bool set_report_descriptor_status(uint_fast16_t length) { return true; }
                __INLINE bool set_physical_descriptor_status(uint_fast8_t index, uint_fast16_t length) { return true; }
                __INLINE bool get_report_status(uint_fast8_t type, uint_fast8_t id, uint_fast16_t length) { return true; }
                __INLINE bool get_idle_status(uint_fast8_t id, uint_fast16_t length) { return true; }
                __INLINE bool get_protocol_status(uint_fast16_t length) { return true; }
                __INLINE bool set_report_status(uint_fast8_t type, uint_fast8_t id, uint_fast16_t length) { return true; }
                __INLINE bool set_idle_status(uint_fast8_t duration, uint_fast8_t id) { return true; }
                __INLINE bool set_protocol_status(uint_fast8_t protocol) { return true; }

                __INLINE constexpr const char& get_hid_descriptor(uint_fast16_t length)
                {
                    const char* np = nullptr;
                    return *np;
                }
                __INLINE const typename DESC::report_t& get_report_descriptor(uint_fast16_t length)
                {
                    static const typename DESC::report_t descriptor;
                    return descriptor;
                }
                __INLINE constexpr const char& get_physical_descriptor(uint_fast8_t index, uint_fast16_t length)
                {
                    const char* np = nullptr;
                    return *np;
                }
                __INLINE void* set_hid_descriptor(uint_fast16_t length) { return nullptr; }
                __INLINE void* set_report_descriptor(uint_fast16_t length) { return nullptr; }
                __INLINE void* set_physical_descriptor(uint_fast8_t index, uint_fast16_t length) { return nullptr; }
                __INLINE const void* get_report(uint_fast8_t type, uint_fast8_t id, uint_fast16_t length) const { return nullptr; }
                __INLINE const void* get_idle(uint_fast8_t id, uint_fast16_t length) const { return nullptr; }
                __INLINE const void* get_protocol(uint_fast16_t length) const { return nullptr; }
                __INLINE void* set_report(uint_fast8_t type, uint_fast8_t id, uint_fast16_t length) { return nullptr; }
                __INLINE bool set_idle(uint_fast8_t duration, uint_fast8_t id) { return true; };
                __INLINE bool set_protocol(uint_fast8_t protocol) { return false; }
            };
        }

        template<template<typename, uint32_t> class HID, typename PARENT, uint32_t PARAM, uint8_t SCLS, uint8_t PROTO, uint8_t STR, uint16_t VER, uint8_t COUNTRY, typename DESC, template<typename, uint32_t> class... EP>
        class if_impl_t;

        template<template<typename, uint32_t> class HID, typename PARENT, uint32_t PARAM, uint8_t SCLS, uint8_t PROTO, uint8_t STR, uint16_t VER, uint8_t COUNTRY, typename DESC, template<typename, uint32_t> class EP>
        class if_impl_t<HID, PARENT, PARAM, SCLS, PROTO, STR, VER, COUNTRY, DESC, EP> : public base::if_impl_t<HID, PARENT, PARAM, SCLS, PROTO, STR, VER, COUNTRY, DESC, EP> {
            using ep_t = typename core::elem_t<0, typename if_impl_t::ep_tuple_t>::type;
            static_assert(ep_t::_ep_type == 3, "the type of endpoint must be INTERRUPT");
            static_assert(ep_t::_ep_dir, "the direction of endpoint must be IN");
        };

        template<template<typename, uint32_t> class HID, typename PARENT, uint32_t PARAM, uint8_t SCLS, uint8_t PROTO, uint8_t STR, uint16_t VER, uint8_t COUNTRY, typename DESC, template<typename, uint32_t> class EP0, template<typename, uint32_t> class EP1>
        class if_impl_t<HID, PARENT, PARAM, SCLS, PROTO, STR, VER, COUNTRY, DESC, EP0, EP1> : public base::if_impl_t<HID, PARENT, PARAM, SCLS, PROTO, STR, VER, COUNTRY, DESC, EP0, EP1> {
            template<uint8_t N> using ep_t = typename core::elem_t<N, typename if_impl_t::ep_tuple_t>::type;
            static_assert(ep_t<0>::_ep_type == 3 && ep_t<1>::_ep_type == 3, "The type of both endpoints must be INTERRUPT");
            static_assert(ep_t<0>::_ep_dir != ep_t<1>::_ep_dir, "The direction of two endpoints must be one is IN, and the other is OUT");
        };
    }
}

#endif  // __USBD_HID_HPP
