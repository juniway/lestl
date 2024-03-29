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

#ifndef __USBD_CORE_HPP
#define __USBD_CORE_HPP

#define NDEBUG
#include <assert.h>
#include <cstdint>
#include <type_traits>
#include <attribute.h>

namespace usbd {
    namespace core {
        namespace helper {
            template<typename PARENT, uint32_t PARAM, template<typename, uint32_t> class... ELEMs>
            struct tuple_t;

            template<typename PARENT, uint32_t PARAM, template<typename, uint32_t> class ELEM, template<typename, uint32_t> class... ELEMs>
            struct tuple_t<PARENT, PARAM, ELEM, ELEMs...> : ELEM<PARENT, PARAM>, tuple_t<PARENT, ELEM<PARENT, PARAM>::_next_param, ELEMs...> {
                static const uint32_t _next_param{ tuple_t<PARENT, ELEM<PARENT, PARAM>::_next_param, ELEMs...>::_next_param };
                static const uint32_t _elem_count{ tuple_t<PARENT, ELEM<PARENT, PARAM>::_next_param, ELEMs...>::_elem_count + 1 };      // sizeof...(ELEMs)
                __INLINE constexpr tuple_t() { }
            };

            template<typename PARENT, uint32_t PARAM, template<typename, uint32_t> class ELEM>
            struct tuple_t<PARENT, PARAM, ELEM> : ELEM<PARENT, PARAM> {
                static const uint32_t _next_param{ ELEM<PARENT, PARAM>::_next_param };
                static const uint32_t _elem_count{ 1 };
                __INLINE constexpr tuple_t() { }
            };

            template<typename PARENT, uint32_t PARAM>
            struct tuple_t<PARENT, PARAM> {
                static const uint32_t _next_param{ PARAM };
                static const uint8_t _if_count{ 0 };
                static const uint8_t _ep_count{ 0 };
                static const uint32_t _elem_count{ 0 };
            };

            template<uint8_t NUM, typename PARENT, uint32_t PARAM, template<typename, uint32_t> class... ELEMs>
            struct elem_t;

            template<uint8_t NUM, typename PARENT, uint32_t PARAM, template<typename, uint32_t> class ELEM, template<typename, uint32_t> class... ELEMs>
            struct elem_t<NUM, PARENT, PARAM, ELEM, ELEMs...> : elem_t<NUM - 1, PARENT, ELEM<PARENT, PARAM>::_next_param, ELEMs...> { };

            template<typename PARENT, uint32_t PARAM, template<typename, uint32_t> class ELEM, template<typename, uint32_t> class... ELEMs>
            struct elem_t<0, PARENT, PARAM, ELEM, ELEMs...> { using type = ELEM<PARENT, PARAM>; };

            template<typename PARENT, uint32_t PARAM>
            struct elem_t<0, PARENT, PARAM> { using type = void; };
        }

        template<typename PARENT, uint32_t PARAM, template<typename, uint32_t> class... ELEMs>
        struct tuple_t : helper::tuple_t<PARENT, PARAM, ELEMs...> {
            using core_tuple_t = tuple_t;
            static const uint8_t _if_count{ ((helper::tuple_t<PARENT, PARAM, ELEMs...>::_next_param >> 8) & 0xf) - ((PARAM >> 8) & 0xff) };
            static const uint8_t _ep_count{ (helper::tuple_t<PARENT, PARAM, ELEMs...>::_next_param & 0xff) - (PARAM & 0xff) };
        };

        template<uint8_t, typename T>
        struct elem_t { using type = T; };

        template<uint8_t NUM, typename PARENT, uint32_t PARAM, template<typename, uint32_t> class... ELEMs>
        struct elem_t<NUM, tuple_t<PARENT, PARAM, ELEMs...>> : helper::elem_t<NUM, PARENT, PARAM, ELEMs...> { };

        template<uint8_t NUM, typename USB>
        struct if_elem_t;

        template<uint8_t NUM, typename PARENT, uint32_t PARAM, template<typename, uint32_t> class ELEM, template<typename, uint32_t> class... ELEMs>
        struct if_elem_t<NUM, tuple_t<PARENT, PARAM, ELEM, ELEMs...>> : std::conditional<
                                (ELEM<PARENT, PARAM>::_if_count > NUM),
                                typename std::conditional<
                                    ELEM<PARENT, PARAM>::_if_count == 1,
                                    elem_t<0, ELEM<PARENT, PARAM>>,
                                    elem_t<NUM, typename ELEM<PARENT, PARAM>::core_tuple_t>
                                >::type,
                                if_elem_t<NUM - ELEM<PARENT, PARAM>::_if_count, tuple_t<PARENT, ELEM<PARENT, PARAM>::_next_param, ELEMs...>>
                            >::type { };

        template<uint8_t NUM, typename PARENT, uint32_t PARAM, template<typename, uint32_t> class ELEM>
        struct if_elem_t<NUM, tuple_t<PARENT, PARAM, ELEM>> : elem_t<NUM, typename ELEM<PARENT, PARAM>::core_tuple_t> { };

        template<typename PARENT, uint32_t PARAM, template<typename, uint32_t> class ELEM>
        struct if_elem_t<0, tuple_t<PARENT, PARAM, ELEM>> : std::conditional<
                                ELEM<PARENT, PARAM>::_if_count == 1,
                                elem_t<0, ELEM<PARENT, PARAM>>,
                                elem_t<0, typename ELEM<PARENT, PARAM>::core_tuple_t>
                            >::type { };

        template<typename PARENT, typename ELEM = void>
        struct root_t : root_t<typename PARENT::_parent_t, PARENT> { };

        template<typename ELEM>
        struct root_t<void, ELEM> { using type = ELEM; };
    }

    enum class langid_t : uint16_t {
        Afrikaans = 0x0436,
        Albanian = 0x041c,
        Arabic_SaudiArabia = 0x0401,
        Arabic_Iraq = 0x0801,
        Arabic_Egypt = 0x0c01,
        Arabic_Libya = 0x1001,
        Arabic_Algeria = 0x1401,
        Arabic_Morocco = 0x1801,
        Arabic_Tunisia = 0x1c01,
        Arabic_Oman = 0x2001,
        Arabic_Yemen = 0x2401,
        Arabic_Syria = 0x2801,
        Arabic_Jordan = 0x2c01,
        Arabic_Lebanon = 0x3001,
        Arabic_Kuwait = 0x3401,
        Arabic_UAE = 0x3801,
        Arabic_Bahrain = 0x3c01,
        Arabic_Qatar = 0x4001,
        Armenian = 0x042b,
        Assamese = 0x044d,
        Azeri_Latin = 0x042c,
        Azeri_Cyrillic = 0x082c,
        Basque = 0x042d,
        Belarussian = 0x0423,
        Bengali = 0x0445,
        Bulgarian = 0x0402,
        Burmese = 0x0455,
        Catalan = 0x0403,
        Chinese_Taiwan = 0x0404,
        Chinese_PRC = 0x0804,
        Chinese_HongKong = 0x0c04,
        Chinese_Singapore = 0x1004,
        Chinese_Macau = 0x1404,
        Croatian = 0x041a,
        Czech = 0x0405,
        Danish = 0x0406,
        Dutch_Netherlands = 0x0413,
        Dutch_Belgium = 0x0813,
        English_UnitedStates = 0x0409,
        English_UnitedKingdom = 0x0809,
        English_Australian = 0x0c09,
        English_Canadian = 0x1009,
        English_NewZealand = 0x1409,
        English_Ireland = 0x1809,
        English_SouthAfrica = 0x1c09,
        English_Jamaica = 0x2009,
        English_Caribbean = 0x2409,
        English_Belize = 0x2809,
        English_Trinidad = 0x2c09,
        English_Zimbabwe = 0x3009,
        English_Philippines = 0x3409,
        Estonian = 0x0425,
        Faeroese = 0x0438,
        Farsi = 0x0429,
        Finnish = 0x040b,
        French_Standard = 0x040c,
        French_Belgian = 0x080c,
        French_Canadian = 0x0c0c,
        French_Switzerland = 0x100c,
        French_Luxembourg = 0x140c,
        French_Monaco = 0x180c,
        Georgian = 0x0437,
        German_Standard = 0x0407,
        German_Switzerland = 0x0807,
        German_Austria = 0x0c07,
        German_Luxembourg = 0x1007,
        German_Liechtenstein = 0x1407,
        Greek = 0x0408,
        Gujarati = 0x0447,
        Hebrew = 0x040d,
        Hindi = 0x0439,
        Hungarian = 0x040e,
        Icelandic = 0x040f,
        Indonesian = 0x0421,
        Italian_Standard = 0x0410,
        Italian_Switzerland = 0x0810,
        Japanese = 0x0411,
        Kannada = 0x044b,
        Kashmiri_India = 0x0860,
        Kazakh = 0x043f,
        Konkani = 0x0457,
        Korean = 0x0412,
        Korean_Johab = 0x0812,
        Latvian = 0x0426,
        Lithuanian = 0x0427,
        Lithuanian_Classic = 0x0827,
        Macedonian = 0x042f,
        Malay_Malaysian = 0x043e,
        Malay_BruneiDarussalam = 0x083e,
        Malayalam = 0x044c,
        Manipuri = 0x0458,
        Marathi = 0x044e,
        Nepali_India = 0x0861,
        Norwegian_Bokmal = 0x0414,
        Norwegian_Nynorsk = 0x0814,
        Oriya = 0x0448,
        Polish = 0x0415,
        Portuguese_Brazil = 0x0416,
        Portuguese_Standard = 0x0816,
        Punjabi = 0x0446,
        Romanian = 0x0418,
        Russian = 0x0419,
        Sanskrit = 0x044f,
        Serbian_Cyrillic = 0x0c1a,
        Serbian_Latin = 0x081a,
        Sindhi = 0x0459,
        Slovak = 0x041b,
        Slovenian = 0x0424,
        Spanish_TraditionalSort = 0x040a,
        Spanish_Mexican = 0x080a,
        Spanish_ModernSort = 0x0c0a,
        Spanish_Guatemala = 0x100a,
        Spanish_CostaRica = 0x140a,
        Spanish_Panama = 0x180a,
        Spanish_DominicanRepublic = 0x1c0a,
        Spanish_Venezuela = 0x200a,
        Spanish_Colombia = 0x240a,
        Spanish_Peru = 0x280a,
        Spanish_Argentina = 0x2c0a,
        Spanish_Ecuador = 0x300a,
        Spanish_Chile = 0x340a,
        Spanish_Uruguay = 0x380a,
        Spanish_Paraguay = 0x3c0a,
        Spanish_Bolivia = 0x400a,
        Spanish_ElSalvador = 0x440a,
        Spanish_Honduras = 0x480a,
        Spanish_Nicaragua = 0x4c0a,
        Spanish_PuertoRico = 0x500a,
        Sutu = 0x0430,
        Swahili_Kenya = 0x0441,
        Swedish = 0x041d,
        Swedish_Finland = 0x081d,
        Tamil = 0x0449,
        Tatar_Tatarstan = 0x0444,
        Telugu = 0x044a,
        Thai = 0x041e,
        Turkish = 0x041f,
        Ukrainian = 0x0422,
        Urdu_Pakistan = 0x0420,
        Urdu_India = 0x0820,
        Uzbek_Latin = 0x0443,
        Uzbek_Cyrillic = 0x0843,
        Vietnamese = 0x042a,
        HID_UsageDataDescriptor = 0x04ff,
        HID_VendorDefined1 = 0xf0ff,
        HID_VendorDefined2 = 0xf4ff,
        HID_VendorDefined3 = 0xf8ff,
        HID_VendorDefined4 = 0xfcff
    };

    enum : uint8_t {
        CS_UNDEFINED = 0x20,
        CS_DEVICE = 0x21,
        CS_CONFIGURATION = 0x22,
        CS_STRING = 0x23,
        CS_INTERFACE = 0x24,
        CS_ENDPOINT = 0x25
    };

    constexpr uint_fast8_t operator "" _mA(uint64_t current)
    {
        return (current + 1) / 2;
    }

    template<uint8_t NUM, typename T>
    __INLINE constexpr typename core::elem_t<NUM, typename T::core_tuple_t>::type& get_elem(T& t)
    {
        return t;
    }

    template<typename T>
    __INLINE constexpr typename core::root_t<typename T::_parent_t>::type& get_usb(T& t)
    {
        return *static_cast<typename core::root_t<typename T::_parent_t>::type*>(&t);
    }

    template<uint8_t NUM, typename USB>
    __INLINE constexpr typename core::if_elem_t<NUM, typename USB::core_tuple_t>::type& get_if(USB& t)
    {
        return t;
    }

    namespace core {
        enum {
            DESC_DEVICE = 1,
            DESC_CONFIG,
            DESC_STRING,
            DESC_INTERFACE,
            DESC_ENDPOINT,
            DESC_IAD = 11
        };

        template<typename...>
        struct descriptor_t;

        template<typename CLSD>
        struct __attribute__((packed)) descriptor_t<CLSD> : CLSD { static const uint8_t _count{ 0 }; };

        template<typename PARENT, uint32_t PARAM, template<typename, uint32_t> class ELEM>
        struct __attribute__((packed)) descriptor_t<ELEM<PARENT, PARAM>> : ELEM<PARENT, PARAM>::descriptor_t { static const uint8_t _count{ 1 }; };

        template<typename ELEM, typename... ELEMs>
        struct __attribute__((packed)) descriptor_t<ELEM, ELEMs...> : descriptor_t<ELEM>, descriptor_t<ELEMs...> {
            static const uint8_t _count{ descriptor_t<ELEM>::_count + descriptor_t<ELEMs...>::_count };
        };

        template<typename PARENT, uint32_t PARAM, template<typename, uint32_t> class ELEM, template<typename, uint32_t> class... ELEMs>
        struct __attribute__((packed)) descriptor_t<core::tuple_t<PARENT, PARAM, ELEM, ELEMs...>>
                : descriptor_t<ELEM<PARENT, PARAM>>, descriptor_t<core::tuple_t<PARENT, ELEM<PARENT, PARAM>::_next_param, ELEMs...>> {
            static const uint8_t _count{ descriptor_t<ELEM<PARENT, PARAM>>::_count + descriptor_t<core::tuple_t<PARENT, ELEM<PARENT, PARAM>::_next_param, ELEMs...>>::_count };
        };

        namespace descriptor {
            struct empty_t { };

            template<typename DESC, typename EP>
            struct __attribute__((packed)) std_ep_t {
                uint8_t bLength{ sizeof(DESC) };
                uint8_t bDescriptorType{ DESC_ENDPOINT };
                uint8_t bEndpointAddress{ (uint8_t(EP::_ep_dir) << 7) + EP::_ep_num };
                uint8_t bmAttributes{ EP::_ep_attr };
                uint16_t wMaxPacketSize{ EP::_ep_pktsz };
                uint8_t bInterval{ EP::_ep_interval };
            };

            template<typename STDD, typename CLSD = empty_t>
            struct __attribute__((packed)) hierarchical_ep_t : STDD, CLSD { };

            template<typename STDD>
            struct template_if_t;

            template<template<typename, uint8_t, uint8_t, uint8_t> class STDD, typename IF, uint8_t ALT, uint8_t EPNUM, uint8_t STR>
            struct __attribute__((packed)) template_if_t<STDD<IF, ALT, EPNUM, STR>> {
                uint8_t bLength{ sizeof(STDD<IF, ALT, EPNUM, STR>) };
                uint8_t bDescriptorType{ DESC_INTERFACE };
                uint8_t bInterfaceNumber{ IF::_if_num };
                uint8_t bAlternateSetting{ ALT };
                uint8_t bNumEndpoints{ EPNUM };//IF::_ep_count
                uint8_t bInterfaceClass{ IF::_class };
                uint8_t bInterfaceSubClass{ IF::_subclass };
                uint8_t bInterfaceProtocol{ IF::_protocol };
                uint8_t iInterface{ STR };
            };

            template<typename IF, uint8_t ALT, uint8_t EPNUM, uint8_t STR>
            struct __attribute__((packed)) std_if_t : template_if_t<std_if_t<IF, ALT, EPNUM, STR>> { };
        }

        template<template<typename, uint8_t, uint8_t, uint8_t> class STDD, typename IF, uint8_t ALT, uint8_t STR>
        struct __attribute__((packed)) descriptor_t<STDD<IF, ALT, 0, STR>> : descriptor::template_if_t<STDD<IF, ALT, 0, STR>> { };

        template<template<typename, uint8_t, uint8_t, uint8_t> class STDD, typename IF, uint8_t ALT, uint8_t STR, typename... ELEMs>
        struct __attribute__((packed)) descriptor_t<STDD<IF, ALT, 0, STR>, ELEMs...> : descriptor::template_if_t<STDD<IF, ALT, descriptor_t<ELEMs...>::_count, STR>>, descriptor_t<ELEMs...> { };

        template<template<typename, uint8_t, uint8_t, uint8_t> class STDD, typename IF, uint8_t ALT, uint8_t STR, typename PARENT, uint32_t PARAM, template<typename, uint32_t> class... EPs>
        struct __attribute__((packed)) descriptor_t<STDD<IF, ALT, 0, STR>, core::tuple_t<PARENT, PARAM, EPs...>> : descriptor::template_if_t<STDD<IF, ALT, descriptor_t<core::tuple_t<PARENT, PARAM, EPs...>>::_count, STR>>, descriptor_t<core::tuple_t<PARENT, PARAM, EPs...>> { };

        template<template<typename, uint8_t, uint8_t, uint8_t> class STDD, typename IF, uint8_t ALT, uint8_t STR, typename CLSD, typename PARENT, uint32_t PARAM, template<typename, uint32_t> class... EPs>
        struct __attribute__((packed)) descriptor_t<STDD<IF, ALT, 0, STR>, CLSD, core::tuple_t<PARENT, PARAM, EPs...>> : descriptor::template_if_t<STDD<IF, ALT, descriptor_t<core::tuple_t<PARENT, PARAM, EPs...>>::_count, STR>>, CLSD, descriptor_t<core::tuple_t<PARENT, PARAM, EPs...>> { };

        enum request_type_t : uint8_t {
            DEVICE_STANDARD_OUT = 0,
            INTERFACE_STANDARD_OUT = 1,
            ENDPOINT_STANDARD_OUT = 2,

            DEVICE_CLASS_OUT = 0x20,
            INTERFACE_CLASS_OUT = 0x21,
            ENDPOINT_CLASS_OUT = 0x22,

            DEVICE_VENDOR_OUT = 0x40,
            INTERFACE_VENDOR_OUT = 0x41,
            ENDPOINT_VENDOR_OUT = 0x42,

            DEVICE_STANDARD_IN = 0x80,
            INTERFACE_STANDARD_IN = 0x81,
            ENDPOINT_STANDARD_IN = 0x82,

            DEVICE_CLASS_IN = 0xa0,
            INTERFACE_CLASS_IN = 0xa1,
            ENDPOINT_CLASS_IN = 0xa2,

            DEVICE_VENDOR_IN = 0xc0,
            INTERFACE_VENDOR_IN = 0xc1,
            ENDPOINT_VENDOR_IN = 0xc2,
        };

        enum class request_t {          // Standard Request
            GET_STATUS,
            CLEAR_FEATURE,

            SET_FEATURE = 3,

            SET_ADDRESS = 5,
            GET_DESCRIPTOR,
            SET_DESCRIPTOR,
            GET_CONFIGURATION,
            SET_CONFIGURATION,
            GET_INTERFACE,
            SET_INTERFACE,
            SYNCH_FRAME
        };

        namespace transfer {
            namespace base {
                template<template<typename, uint32_t> class EP, typename PARENT, uint32_t PARAM, uint8_t TYPE, bool DIR, uint8_t ATTR, uint8_t INTERVAL, typename HAL>
                class ep_impl_t : public HAL {
                protected:
                    using ep_t = EP<PARENT, PARAM>;

                public:
                    enum { HALT = 0 };
                    static const uint32_t _param{ PARAM };
                    static const uint8_t _ep_num{ PARAM & 0xff };
                    static const uint8_t _ep_type{ TYPE };
                    static const bool _ep_dir{ DIR };
                    static const uint8_t _ep_attr{ ATTR + ep_impl_t::_ep_type };
                    static const uint8_t _ep_interval{ INTERVAL };
                    using super_t = ep_impl_t;
                    using _parent_t = PARENT;
                    static const uint32_t _next_param{ PARAM + 1 };
                    struct __attribute__((packed)) std_descriptor_t : core::descriptor::std_ep_t<typename ep_t::std_descriptor_t, ep_t> { };
                    struct class_descriptor_t { };
                    struct __attribute__((packed)) descriptor_t : ep_t::std_descriptor_t, ep_t::class_descriptor_t { };

                protected:
                    __INLINE constexpr ep_impl_t() { }
                    __INLINE PARENT& parent()
                    {
                        return *static_cast<PARENT*>(this);
                    }

                public:
                    __INLINE void attach() { }
                    __INLINE void detach() { }
                    __INLINE void reset() { }
                    __INLINE void suspend() { }
                    __INLINE void resume() { }
                    __INLINE void config()
                    {
                        HAL::config(_ep_type, _ep_num, _ep_dir);
                    }
                    __PURE bool event();
//                  __INLINE void set_stall() { HAL::set_stall(); }

                    __INLINE bool standard_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length)
                    {
                        auto& ep = *static_cast<ep_t*>(this);
                        switch (request_t(request)) {
                        case request_t::CLEAR_FEATURE:
                            return ep.clear_feature_setup(value);
                        case request_t::SET_FEATURE:
                            return ep.set_feature_setup(value);
                        default:
                            assert(false);
                            return false;
                        }
                    }
                    __INLINE bool standard_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length)
                    {
                        auto& ep = *static_cast<ep_t*>(this);
                        assert(length == sizeof(uint16_t));
                        switch (request_t(request)) {
                        case request_t::GET_STATUS:
                            assert(value == 0);
                            assert(length == 2);
                            return ep.get_status_setup(length);
                        case request_t::SYNCH_FRAME:
                            assert(value == 0);
                            assert(length == 2);
                            return ep.synch_frame_setup(length);
                        default:
                            assert(false);
                            return false;
                        }
                    }
                    __INLINE bool standard_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length)
                    {
                        get_usb(*this).out();
                        return true;
                    }
                    __INLINE bool standard_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length)
                    {
                        auto& ep = *static_cast<ep_t*>(this);
                        switch (request_t(request)) {
                        case request_t::GET_STATUS:
                            return ep.get_status_data(length);
                        case request_t::SYNCH_FRAME:
                            return ep.synch_frame_data(length);
                        default:
                            get_usb(ep).in();
                            return true;
                        }
                    }
                    __INLINE bool standard_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length)
                    {
                        auto& ep = *static_cast<ep_t*>(this);
                        switch (request_t(request)) {
                        case request_t::GET_STATUS:
                            return ep.get_status_status(length);
                        case request_t::SYNCH_FRAME:
                            return ep.synch_frame_status(length);
                        default:
                            return true;
                        }
                    }
                    __INLINE bool standard_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length)
                    {
                        auto& ep = *static_cast<ep_t*>(this);
                        switch (request_t(request)) {
                        case request_t::CLEAR_FEATURE:
                            return ep.clear_feature_status(value);
                        case request_t::SET_FEATURE:
                            return ep.set_feature_status(value);
                        default:
                            return true;
                        }
                    }
                    __INLINE bool class_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length)
                    {
                        get_usb(*this).out();
                        return true;
                    }
                    __INLINE bool class_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length)
                    {
                        get_usb(*this).in();
                        return true;
                    }
                    __INLINE bool vendor_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length)
                    {
                        get_usb(*this).out();
                        return true;
                    }
                    __INLINE bool vendor_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length)
                    {
                        get_usb(*this).in();
                        return true;
                    }
                    __INLINE bool get_status_setup(uint_fast16_t length)
                    {
                        auto& ep = *static_cast<ep_t*>(this);
                        const void* buffer = ep.get_status(length);
                        if (buffer != nullptr) {
                            get_usb(ep).in(buffer, length, length);
                            return true;
                        }
                        return false;
                    }
                    __INLINE bool clear_feature_setup(uint_fast8_t selector)
                    {
                        auto& ep = *static_cast<ep_t*>(this);
                        if (ep.clear_feature(selector)) {
                            get_usb(ep).out(nullptr, 0);
                            return true;
                        }
                        return false;
                    }
                    __INLINE bool synch_frame_setup(uint_fast16_t length)
                    {
                        auto& ep = *static_cast<ep_t*>(this);
                        const void* buffer = ep.synch_frame(length);
                        if (buffer != nullptr) {
                            get_usb(ep).in(buffer, length, length);
                            return true;
                        }
                        return false;
                    }
                    __INLINE bool set_feature_setup(uint_fast8_t selector)
                    {
                        auto& ep = *static_cast<ep_t*>(this);
                        if (ep.set_feature(selector)) {
                            get_usb(ep).out(nullptr, 0);
                            return true;
                        }
                        return false;
                    }
                    __INLINE bool get_status_data(uint_fast16_t length)
                    {
                        get_usb(*this).in();
                        return true;
                    }
                    __INLINE bool synch_frame_data(uint_fast16_t length)
                    {
                        get_usb(*this).in();
                        return true;
                    }
                    __INLINE bool class_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length) { return false; }
                    __INLINE bool class_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length) { return false; }
                    __INLINE bool vendor_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length) { return false; }
                    __INLINE bool vendor_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length) { return false; }

                    __INLINE bool class_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length) { return true; }
                    __INLINE bool class_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length) { return true; }
                    __INLINE bool vendor_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length) { return true; }
                    __INLINE bool vendor_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t length) { return true; }

                    __INLINE bool get_status_status(uint_fast16_t length) { return true; }
                    __INLINE bool clear_feature_status(uint_fast8_t selector) { return true; }
                    __INLINE bool set_feature_status(uint_fast8_t selector) { return true; }
                    __INLINE bool synch_frame_status(uint_fast16_t length) { return true; }

                    __INLINE const void* get_status(uint_fast16_t length) { return nullptr; }
                    __INLINE bool clear_feature(uint_fast8_t selector) { return false; }
                    __INLINE bool set_feature(uint_fast8_t selector) { return false; }
                    __INLINE const void* synch_frame(uint_fast16_t length) { return nullptr; }

                };

                namespace in {
                    template<template<typename, uint32_t> class EP, typename PARENT, uint32_t PARAM, uint8_t TYPE, uint8_t ATTR, uint8_t INTERVAL, typename HAL>
                    class ep_impl_t : public base::ep_impl_t<EP, PARENT, PARAM, TYPE, true, ATTR, INTERVAL, HAL> {
                    protected:
                        __INLINE constexpr ep_impl_t() { }

                    public:
                        __INLINE bool event()
                        {
                            if (ep_impl_t::in())
                                this->parent().write_complete();
                            return true;
                        }
                        __INLINE bool write(const void* buffer, uint_fast16_t length)
                        {
                            return ep_impl_t::in(buffer, length, length);
                        }

                    private:
                        using ep_impl_t::super_t::out;
                    };
                }

                namespace out {
                    template<template<typename, uint32_t> class EP, typename PARENT, uint32_t PARAM, uint8_t TYPE, uint8_t ATTR, uint8_t INTERVAL, typename HAL>
                    class ep_impl_t : public base::ep_impl_t<EP, PARENT, PARAM, TYPE, false, ATTR, INTERVAL, HAL> {
                    protected:
                        __INLINE constexpr ep_impl_t() { }

                    public:
                        __INLINE bool event()
                        {
                            uint_fast16_t length = ep_impl_t::out();
                            if (length != 0)
                                this->parent().read_complete(length);
                            return true;
                        }
                        __INLINE void read(void* buffer, uint_fast16_t length)
                        {
                            ep_impl_t::out(buffer, length);
                        }

                    private:
                        using ep_impl_t::super_t::in;
                    };
                }
            }
            namespace control {
                namespace base {
                    class ep_impl_t : public hal::transfer::control::ep_impl_t {
                    public:
                        __INLINE ep_impl_t() { }
                        __PURE bool setup(uint_fast8_t type, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length);
                        __PURE void transact_in(uint_fast8_t type, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length);
                        __PURE void transact_out(uint_fast8_t type, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length);
                    };
                }

                template<template<typename, uint32_t> class EP, typename PARENT, uint32_t PARAM>
                using ep_impl_t = transfer::base::ep_impl_t<EP, PARENT, PARAM, 0, true, 0, 0, base::ep_impl_t>;
            }
            namespace ctrl = control;

            namespace isochronous {
                enum sync_t {       // Synchronisation Type
                    NO,             // No Synchonisation
                    ASYNC,          // Asynchronous
                    ADAPT,          // Adaptive
                    SYNC            // Synchronous
                };

                enum usage_t {      // Usage Type
                    DATA,           // Data Endpoint
                    FEEDBACK,       // Feedback Endpoint
                    EXP_FB          // Explicit Feedback Data Endpoint
                };

                namespace in {
                    template<template<typename, uint32_t> class EP, typename PARENT, uint32_t PARAM, sync_t STYPE, usage_t UTYPE>
                    using ep_impl_t = base::in::ep_impl_t<EP, PARENT, PARAM, 1, (uint8_t(UTYPE) << 4) + (uint8_t(STYPE) << 2), 1, hal::transfer::isochronous::ep_impl_t>;
                }
                namespace out {
                    template<template<typename, uint32_t> class EP, typename PARENT, uint32_t PARAM, sync_t STYPE, usage_t UTYPE>
                    using ep_impl_t = base::out::ep_impl_t<EP, PARENT, PARAM, 1, (uint8_t(UTYPE) << 4) + (uint8_t(STYPE) << 2), 1, hal::transfer::isochronous::ep_impl_t>;
                }
            }
            namespace iso = isochronous;
            namespace bulk {
                namespace in {
                    template<template<typename, uint32_t> class EP, typename PARENT, uint32_t PARAM>
                    using ep_impl_t = base::in::ep_impl_t<EP, PARENT, PARAM, 2, 0, 0, hal::transfer::bulk::ep_impl_t>;
                }
                namespace out {
                    template<template<typename, uint32_t> class EP, typename PARENT, uint32_t PARAM>
                    using ep_impl_t = base::out::ep_impl_t<EP, PARENT, PARAM, 2, 0, 0, hal::transfer::bulk::ep_impl_t>;
                }
            }
            namespace interrupt {
                namespace in {
                    template<template<typename, uint32_t> class EP, typename PARENT, uint32_t PARAM, uint8_t INTERVAL>
                    using ep_impl_t = base::in::ep_impl_t<EP, PARENT, PARAM, 3, 0, INTERVAL, hal::transfer::interrupt::ep_impl_t>;
                }
                namespace out {
                    template<template<typename, uint32_t> class EP, typename PARENT, uint32_t PARAM, uint8_t INTERVAL>
                    using ep_impl_t = base::out::ep_impl_t<EP, PARENT, PARAM, 3, 0, INTERVAL, hal::transfer::interrupt::ep_impl_t>;
                }
            }
            namespace intr = interrupt;
        }

        template<template<typename, uint32_t> class IF, typename PARENT, uint32_t PARAM,
                uint8_t CLS, uint8_t SCLS, uint8_t PROTO, uint8_t STR, template<typename, uint32_t> class... EPs>
        class if_impl_t : public tuple_t<IF<PARENT, PARAM>, PARAM, EPs...> {
        protected:
            using if_t = IF<PARENT, PARAM>;
            using super_t = if_impl_t;
            using ep_tuple_t = typename if_impl_t::tuple_t;

            __INLINE constexpr if_impl_t() { }

        public:
            using _parent_t = PARENT;
            static const uint8_t _if_num{ (PARAM >> 8) & 0xff };
            static const uint8_t _if_count{ 1 };
            static const uint32_t _next_param{ if_impl_t::tuple_t::_next_param + 0x100 };
            static const uint8_t _class{ CLS };
            static const uint8_t _subclass{ SCLS };
            static const uint8_t _protocol{ PROTO };

            using std_descriptor_t = descriptor::std_if_t<if_t, 0, 0, STR>;
            struct class_descriptor_t { };
            struct __attribute__((packed)) descriptor_t : core::descriptor_t<typename if_t::std_descriptor_t, typename if_t::class_descriptor_t, typename if_impl_t::tuple_t> { };

            __INLINE PARENT& parent()
            {
                return *static_cast<PARENT*>(this);
            }
            __INLINE bool standard_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
            {
                auto& rif = *static_cast<if_t*>(this);
                switch (request_t(request)) {
                case request_t::CLEAR_FEATURE:
                    return rif.clear_feature_setup(value);
                case request_t::SET_FEATURE:
                    return rif.set_feature_setup(value);
                case request_t::SET_DESCRIPTOR:
                    return rif.set_descriptor_setup(value, length);
                case request_t::SET_INTERFACE:
                    return rif.set_interface_setup(value);
                default:
                    assert(false);
                    return false;
                }
            }
            __INLINE bool standard_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
            {
                auto& rif = *static_cast<if_t*>(this);
                switch (request_t(request)) {
                case request_t::GET_STATUS:
                    assert(length == sizeof(uint16_t));
                    return rif.get_status_setup(length);
                case request_t::GET_DESCRIPTOR:
                    assert(length != 0);
                    return rif.get_descriptor_setup(value, length);
                case request_t::GET_INTERFACE:
                    assert(length == sizeof(uint8_t));
                    return rif.get_interface_setup(length);
                default:
                    assert(false);
                    return false;
                }
            }
            __INLINE bool standard_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
            {
                auto& rif = *static_cast<if_t*>(this);
                switch (request_t(request)) {
                case request_t::GET_STATUS:
                    return rif.get_status_data(length);
                case request_t::GET_DESCRIPTOR:
                    return rif.get_descriptor_data(value, length);
                case request_t::GET_INTERFACE:
                    return rif.get_interface_data(length);
                default:
                    assert(false);
                    get_usb(rif).in();
                    return true;
                }
            }
            __INLINE bool standard_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
            {
                auto& rif = *static_cast<if_t*>(this);
                switch (request_t(request)) {
                case request_t::SET_DESCRIPTOR:
                    return rif.set_descriptor_data(value, length);
                default:
                    assert(false);
                    get_usb(rif).out();
                    return true;
                }
            }
            __INLINE bool standard_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
            {
                auto& rif = *static_cast<if_t*>(this);
                switch (request_t(request)) {
                case request_t::CLEAR_FEATURE:
                    return rif.clear_feature_status(value);
                case request_t::SET_FEATURE:
                    return rif.set_feature_status(value);
                case request_t::SET_DESCRIPTOR:
                    return rif.set_descriptor_status(value, length);
                case request_t::SET_INTERFACE:
                    return rif.set_interface_status(value);
                default:
                    assert(false);
                    return true;
                }
            }
            __INLINE bool standard_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
            {
                auto& rif = *static_cast<if_t*>(this);
                switch (request_t(request)) {
                case request_t::GET_STATUS:
                    return rif.get_status_status(length);
                case request_t::GET_DESCRIPTOR:
                    return rif.get_descriptor_status(value, length);
                case request_t::GET_INTERFACE:
                    return rif.get_interface_status(length);
                default:
                    assert(false);
                    return true;
                }
            }
            __INLINE bool class_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length) { return false; }
            __INLINE bool class_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length) { return false; }
            __INLINE bool vendor_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length) { return false; }
            __INLINE bool vendor_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length) { return false; }
            __INLINE bool class_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
            {
                get_usb(*this).in();
                return true;
            }
            __INLINE bool class_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
            {
                get_usb(*this).out();
                return true;
            }
            __INLINE bool class_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length) { return true; }
            __INLINE bool class_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length) { return true; }
            __INLINE bool vendor_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
            {
                get_usb(*this).in();
                return true;
            }
            __INLINE bool vendor_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length)
            {
                get_usb(*this).out();
                return true;
            }
            __INLINE bool vendor_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length) { return true; }
            __INLINE bool vendor_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length) { return true; }
            __INLINE bool get_descriptor_setup(uint_fast16_t value, uint_fast16_t length)
            {
                auto& rif = *static_cast<if_t*>(this);
                auto& descriptor = rif.get_descriptor(value, length);
                if (&descriptor != nullptr) {
                    get_usb(rif).in(&descriptor, sizeof(descriptor), length);
                    return true;
                }
                return false;
            }
            __INLINE bool get_descriptor_data(uint_fast16_t value, uint_fast16_t length)
            {
                get_usb(*this).in();
                return true;
            }
            __INLINE bool set_descriptor_setup(uint_fast16_t value, uint_fast16_t length)
            {
                auto& rif = *static_cast<if_t*>(this);
                void* descriptor = rif.set_descriptor(value, length);
                if (descriptor != nullptr) {
                    get_usb(rif).out(descriptor, length);
                    return true;
                }
                return false;
            }
            __INLINE bool set_descriptor_data(uint_fast16_t value, uint_fast16_t length)
            {
                get_usb(*this).out();
                return true;
            }
            __INLINE bool get_status_setup(uint_fast16_t length)
            {
                auto& rif = *static_cast<if_t*>(this);
                const void* buffer = rif.get_status(length);
                if (buffer != nullptr) {
                    get_usb(rif).in(buffer, length, length);
                    return true;
                }
                return false;
            }
            __INLINE bool get_status_data(uint_fast16_t length)
            {
                get_usb(*this).in();
                return true;
            }
            __INLINE bool clear_feature_setup(uint_fast8_t selector)
            {
                auto& rif = *static_cast<if_t*>(this);
                if (rif.clear_feature(selector)) {
                    get_usb(rif).out(nullptr, 0);
                    return true;
                }
                return false;
            }
            __INLINE bool set_feature_setup(uint_fast8_t selector)
            {
                auto& rif = *static_cast<if_t*>(this);
                if (rif.set_feature(selector)) {
                    get_usb(rif).out(nullptr, 0);
                    return true;
                }
                return false;
            }
            __INLINE bool get_interface_setup(uint_fast16_t length)
            {
                auto& rif = *static_cast<if_t*>(this);
                const uint8_t* interface = rif.get_interface(length);
                if (interface != nullptr) {
                    get_usb(rif).in(interface, length, length);
                    return true;
                }
                return false;
            }
            __INLINE bool get_interface_data(uint_fast16_t length)
            {
                get_usb(*this).in();
                return true;
            }
            __INLINE bool set_interface_setup(uint_fast8_t alternative)
            {
                auto& rif = *static_cast<if_t*>(this);
                if (rif.set_interface(alternative)) {
                    get_usb(rif).out(nullptr, 0);
                    return true;
                }
                return false;
            }
            __INLINE bool get_descriptor_status(uint_fast16_t value, uint_fast16_t length) { return true; }
            __INLINE bool set_descriptor_status(uint_fast16_t value, uint_fast16_t length) { return true; }
            __INLINE bool get_status_status(uint_fast16_t length) { return true; }
            __INLINE bool clear_feature_status(uint_fast8_t selector) { return true; }
            __INLINE bool set_feature_status(uint_fast8_t selector) { return true; }
            __INLINE bool get_interface_status(uint_fast16_t length) { return true; }
            __INLINE bool set_interface_status(uint_fast8_t alternative) { return true; }

            __INLINE uint8_t& get_descriptor(uint_fast16_t value, uint_fast16_t length) { return *static_cast<uint8_t*>(nullptr); }
            __INLINE void* set_descriptor(uint_fast16_t value, uint_fast16_t length) { return nullptr; }
            __INLINE const void* get_status(uint_fast16_t length) { return nullptr; }
            __INLINE bool clear_feature(uint_fast8_t selector) { return false; }
            __INLINE bool set_feature(uint_fast8_t selector) { return false; }
            __INLINE const uint8_t* get_interface(uint_fast16_t length) { return nullptr; }
            __INLINE bool set_interface(uint_fast8_t alternative) { return true; }
            __INLINE void write_complete() { }
            __INLINE void read_complete(uint_fast16_t length) { }

            __INLINE bool ep_standard_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::standard_request_out(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_standard_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::standard_request_in(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_class_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::class_request_out(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_class_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::class_request_in(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_vendor_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::vendor_request_out(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_vendor_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::vendor_request_in(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_standard_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::standard_data_out(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_standard_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::standard_data_in(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_class_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::class_data_out(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_class_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::class_data_in(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_vendor_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::vendor_data_out(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_vendor_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::vendor_data_in(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_standard_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::standard_status_out(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_standard_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::standard_status_in(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_class_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::class_status_out(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_class_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::class_status_in(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_vendor_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::vendor_status_out(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE bool ep_vendor_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                return helper_t<>::vendor_status_in(*static_cast<if_t*>(this), request, value, index, length);
            }
            __INLINE void attach()
            {
                helper_t<>::attach(*static_cast<if_t*>(this));
            }
            __INLINE void detach()
            {
                helper_t<>::detach(*static_cast<if_t*>(this));
            }
            __INLINE void reset()
            {
                helper_t<>::reset(*static_cast<if_t*>(this));
            }
            __INLINE void suspend()
            {
                helper_t<>::suspend(*static_cast<if_t*>(this));
            }
            __INLINE void resume()
            {
                helper_t<>::resume(*static_cast<if_t*>(this));
            }
            __INLINE void config()
            {
                helper_t<>::config(*static_cast<if_t*>(this));
            }
            __INLINE void event(uint_fast8_t index)
            {
                helper_t<>::event(*static_cast<if_t*>(this), index);
            }

        private:
            template<typename T = void, uint8_t N = 0>
            struct helper_t {
                __INLINE static bool standard_request_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).standard_request_out(request, value, length) : helper_t<T, N + 1>::standard_request_out(rif, request, value, index, length);
                }
                __INLINE static bool standard_request_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).standard_request_in(request, value, length) : helper_t<T, N + 1>::standard_request_in(rif, request, value, index, length);
                }
                __INLINE static bool class_request_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).class_request_out(request, value, length) : helper_t<T, N + 1>::class_request_out(rif, request, value, index, length);
                }
                __INLINE static bool class_request_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).class_request_in(request, value, length) : helper_t<T, N + 1>::class_request_in(rif, request, value, index, length);
                }
                __INLINE static bool vendor_request_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).vendor_request_out(request, value, length) : helper_t<T, N + 1>::vendor_request_out(rif, request, value, index, length);
                }
                __INLINE static bool vendor_request_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).vendor_request_in(request, value, length) : helper_t<T, N + 1>::vendor_request_in(rif, request, value, index, length);
                }
                __INLINE static bool standard_data_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).standard_data_out(request, value, length) : helper_t<T, N + 1>::standard_data_out(rif, request, value, index, length);
                }
                __INLINE static bool standard_data_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).standard_data_in(request, value, length) : helper_t<T, N + 1>::standard_data_in(rif, request, value, index, length);
                }
                __INLINE static bool class_data_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).class_data_out(request, value, length) : helper_t<T, N + 1>::class_data_out(rif, request, value, index, length);
                }
                __INLINE static bool class_data_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).class_data_in(request, value, length) : helper_t<T, N + 1>::class_data_in(rif, request, value, index, length);
                }
                __INLINE static bool vendor_data_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).vendor_data_out(request, value, length) : helper_t<T, N + 1>::vendor_data_out(rif, request, value, index, length);
                }
                __INLINE static bool vendor_data_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).vendor_data_in(request, value, length) : helper_t<T, N + 1>::vendor_data_in(rif, request, value, index, length);
                }
                __INLINE static bool standard_status_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).standard_status_out(request, value, length) : helper_t<T, N + 1>::standard_status_out(rif, request, value, index, length);
                }
                __INLINE static bool standard_status_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).standard_status_in(request, value, length) : helper_t<T, N + 1>::standard_status_in(rif, request, value, index, length);
                }
                __INLINE static bool class_status_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).class_status_out(request, value, length) : helper_t<T, N + 1>::class_status_out(rif, request, value, index, length);
                }
                __INLINE static bool class_status_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).class_status_in(request, value, length) : helper_t<T, N + 1>::class_status_in(rif, request, value, index, length);
                }
                __INLINE static bool vendor_status_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).vendor_status_out(request, value, length) : helper_t<T, N + 1>::vendor_status_out(rif, request, value, index, length);
                }
                __INLINE static bool vendor_status_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index == N ? get_elem<N>(rif).vendor_status_in(request, value, length) : helper_t<T, N + 1>::vendor_status_in(rif, request, value, index, length);
                }
                __INLINE static void attach(if_t& rif)
                {
                    get_elem<N>(rif).attach();
                    helper_t<T, N + 1>::attach(rif);
                }
                __INLINE static void detach(if_t& rif)
                {
                    get_elem<N>(rif).detach();
                    helper_t<T, N + 1>::detach(rif);
                }
                __INLINE static void reset(if_t& rif)
                {
                    get_elem<N>(rif).reset();
                    helper_t<T, N + 1>::reset(rif);
                }
                __INLINE static void suspend(if_t& rif)
                {
                    get_elem<N>(rif).suspend();
                    helper_t<T, N + 1>::suspend(rif);
                }
                __INLINE static void resume(if_t& rif)
                {
                    get_elem<N>(rif).resume();
                    helper_t<T, N + 1>::resume(rif);
                }
                __INLINE static void config(if_t& rif)
                {
                    get_elem<N>(rif).config();
                    helper_t<T, N + 1>::config(rif);
                }
                __INLINE static void event(if_t& rif, uint_fast8_t index)
                {
                    if (index == N)
                        get_elem<N>(rif).event();
                    else
                        helper_t<T, N + 1>::event(rif, index);
                }
            };

            template<typename T>
            struct helper_t<T, if_t::_ep_count> {
                __INLINE static bool standard_request_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
                __INLINE static bool standard_request_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
                __INLINE static bool class_request_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
                __INLINE static bool class_request_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
                __INLINE static bool vendor_request_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
                __INLINE static bool vendor_request_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
                // 下面的 *_data_* 函数，并不需要 get_usb(rif).in() 或 get_usb(rif).out()，好像程序并不会编译这里的代码（甚至放一个 while (true) 都没有用，还需要进一步搞清楚。
                __INLINE static bool standard_data_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool standard_data_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool class_data_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool class_data_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool vendor_data_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool vendor_data_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                ///
                __INLINE static bool standard_status_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool standard_status_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool class_status_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool class_status_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool vendor_status_out(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool vendor_status_in(if_t& rif, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static void attach(if_t& rif) { }
                __INLINE static void detach(if_t& rif) { }
                __INLINE static void reset(if_t& rif) { }
                __INLINE static void suspend(if_t& rif) { }
                __INLINE static void resume(if_t& rif) { }
                __INLINE static void config(if_t& rif) { }
                __INLINE static void event(if_t& rif, uint_fast8_t index) { }
            };
        };
        // interface association
        template<template<typename, uint32_t> class FN, typename PARENT, uint32_t PARAM,
                uint8_t CLS, uint8_t SCLS, uint8_t PROTO, uint8_t STR,
                template<typename, uint32_t> class IF, template<typename, uint32_t> class... IFs>
        class fn_impl_t : public tuple_t<FN<PARENT, PARAM>, PARAM, IF, IFs...> {
        protected:
            using fn_t = FN<PARENT, PARAM>;
            using super_t = fn_impl_t;
            using if_tuple_t = typename fn_impl_t::tuple_t;

            __INLINE constexpr fn_impl_t() { }

        public:
            using _parent_t = PARENT;
            static const uint32_t _next_param{ fn_impl_t::tuple_t::_next_param + 0x10000 };
            static const uint8_t _class{ CLS };
            static const uint8_t _subclass{ SCLS };
            static const uint8_t _protocol{ PROTO };
            struct __attribute__((packed)) iad_t {
                uint8_t bLength{ sizeof(typename fn_t::std_descriptor_t) };
                uint8_t bDescriptorType{ DESC_IAD };
                uint8_t bFirstInterface{ (PARAM >> 8) & 0xff };
                uint8_t bInterfaceCount{ fn_impl_t::_if_count };        // sizeof...(IFs) + 1
                uint8_t bFunctionClass{ CLS };
                uint8_t bFunctionSubClass{ SCLS };
                uint8_t bFunctionProtocol{ PROTO };
                uint8_t iFunction{ STR };
            };

            struct std_descriptor_t : std::conditional<STR == 0xff, typename descriptor::empty_t, typename fn_t::iad_t>::type { };
//          using if_descriptor_t = descriptor::tuple_t<fn_t, PARAM, IF, IFs...>;
            using if_descriptor_t = core::descriptor_t<typename fn_impl_t::tuple_t>;
            struct __attribute__((packed)) descriptor_t : fn_t::std_descriptor_t, fn_t::if_descriptor_t { };

            __INLINE PARENT& parent()
            {
                return *static_cast<PARENT*>(this);
            }
            __INLINE void attach()
            {
                helper_t<>::attach(*static_cast<fn_t*>(this));
            }
            __INLINE void detach()
            {
                helper_t<>::detach(*static_cast<fn_t*>(this));
            }
            __INLINE void reset()
            {
                helper_t<>::reset(*static_cast<fn_t*>(this));
            }
            __INLINE void suspend()
            {
                helper_t<>::suspend(*static_cast<fn_t*>(this));
            }
            __INLINE void resume()
            {
                helper_t<>::resume(*static_cast<fn_t*>(this));
            }
            __INLINE void config()
            {
                helper_t<>::config(*static_cast<fn_t*>(this));
            }
            __INLINE void event(uint_fast8_t index)
            {
                helper_t<>::event(*static_cast<fn_t*>(this), index);
            }

        private:
            template<typename T = void, uint8_t N = 0>
            struct helper_t {
                __INLINE static void attach(fn_t& fn)
                {
                    get_elem<N>(fn).attach();
                    helper_t<T, N + 1>::attach(fn);
                }
                __INLINE static void detach(fn_t& fn)
                {
                    get_elem<N>(fn).detach();
                    helper_t<T, N + 1>::detach(fn);
                }
                __INLINE static void reset(fn_t& fn)
                {
                    get_elem<N>(fn).reset();
                    helper_t<T, N + 1>::reset(fn);
                }
                __INLINE static void suspend(fn_t& fn)
                {
                    get_elem<N>(fn).suspend();
                    helper_t<T, N + 1>::suspend(fn);
                }
                __INLINE static void resume(fn_t& fn)
                {
                    get_elem<N>(fn).resume();
                    helper_t<T, N + 1>::resume(fn);
                }
                __INLINE static void config(fn_t& fn)
                {
                    get_elem<N>(fn).config();
                    helper_t<T, N + 1>::config(fn);
                }
                __INLINE static void event(fn_t& fn, uint_fast8_t index)
                {
                    if (index < get_elem<N>(fn)._ep_count)
                        get_elem<N>(fn).event(index);
                    helper_t<T, N + 1>::event(fn, index - get_elem<N>(fn)._ep_count);
                }
            };
            template<typename T>
            struct helper_t<T, fn_t::_elem_count> {
                __INLINE static void attach(fn_t& fn) { }
                __INLINE static void detach(fn_t& fn) { }
                __INLINE static void reset(fn_t& fn) { }
                __INLINE static void suspend(fn_t& fn) { }
                __INLINE static void resume(fn_t& fn) { }
                __INLINE static void config(fn_t& fn) { }
                __INLINE static void event(fn_t& fn, uint_fast8_t index) { }
            };
        };

        template<typename USB,
                uint16_t VER, uint8_t CLS, uint8_t SCLS, uint8_t PROTO, uint16_t VENDOR, uint16_t PRODUCT, uint16_t RELEASE,
                uint8_t MANUSTR, uint8_t PRODSTR, uint8_t SERSTR,
                bool BUSPWR, bool SELFPWR, bool RWAKEUP, uint8_t POWER, uint8_t CFGSTR,
                template<typename, uint32_t> class FN, template<typename, uint32_t> class... FNs>
        class usbd_impl_t : public hal::usbd_impl_t<USB, transfer::control::base::ep_impl_t>, public tuple_t<USB, 1, FN, FNs...> {
            static_assert(BUSPWR && POWER > 0, "the current must be greater than zero");
            static_assert(POWER <= 250, "the current must not exceed 500mA");
            friend class hal::usbd_impl_t<USB, transfer::control::base::ep_impl_t>;

        protected:
            using super_t = usbd_impl_t;
            __INLINE constexpr usbd_impl_t() { }

        public:
            using _parent_t = void;
            using ep0_t = hal::usbd_impl_t<USB, transfer::control::base::ep_impl_t>;
            static const uint8_t _class{ CLS };
            static const uint8_t _subclass{ SCLS };
            static const uint8_t _protocol{ PROTO };
            struct __attribute__((packed)) device_descriptor_t {
                uint8_t bLength{ sizeof(typename USB::device_descriptor_t) };
                uint8_t bDescriptorType{ DESC_DEVICE };
                uint16_t bcdUSB{ VER };
                uint8_t bDeviceClass{ CLS };
                uint8_t bDeviceSubClass{ SCLS };
                uint8_t bDeviceProtocol{ PROTO };
                uint8_t bMaxPacketSize0{ usbd_impl_t::_ep_pktsz };
                uint16_t idVendor{ VENDOR };
                uint16_t idProduct{ PRODUCT };
                uint16_t bcdDevice{ RELEASE };
                uint8_t iManufacture{ MANUSTR };
                uint8_t iProduct{ PRODSTR };
                uint8_t iSerialNumber{ SERSTR };
                uint8_t bNumConfigurations{ 1 };
            };

            struct __attribute__((packed)) config_descriptor_t {
                uint8_t bLength{ sizeof(typename USB::config_descriptor_t) };
                uint8_t bDescriptorType{ DESC_CONFIG };
                uint16_t __attribute__((packed)) wTotalLength{ sizeof(typename USB::hierarchical_descriptor_t) };
                uint8_t bNumInterfaces{ USB::_if_count };
                uint8_t bConfigurationValue{ 1 };
                uint8_t iConfiguration{ CFGSTR };
                uint8_t bmAttributes{ (uint8_t(BUSPWR) << 7) | (uint8_t(SELFPWR) << 6) | (uint8_t(RWAKEUP) << 5) };
                uint8_t bMaxPower{ POWER };
            };

            struct __attribute__((packed)) hierarchical_descriptor_t : USB::config_descriptor_t, descriptor_t<typename usbd_impl_t::tuple_t> { };

            enum {          // State
                STATE_DETACHED = 0x00,
                STATE_ATTACHED = 0x01,
                STATE_POWERED = 0x03,
                STATE_DEFAULT = 0x07,
                STATE_ADDRESS = 0x0f,
                STATE_CONFIGURED = 0x1f,

                STATE_FLAG_ATTACHED = 0x01,
                STATE_FLAG_POWERED = 0x02,
                STATE_FLAG_DEFAULT = 0x04,
                STATE_FLAG_ADDRESS = 0x08,
                STATE_FLAG_CONFIGURED = 0x10,
                STATE_FLAG_SUSPENDED = 0x20
            };

        public:
            using ep0_t::in;
            using ep0_t::out;
            using ep0_t::set_stall;
            using ep0_t::clear_stall;
            using ep0_t::event;

            __INLINE uint_fast8_t get_state() const
            {
                return state;
            }

        protected:
            __INLINE bool attach()
            {
                if ((state & STATE_FLAG_ATTACHED) != 0)
                    return false;
                state = STATE_ATTACHED;
                auto& usb = *static_cast<USB*>(this);
                elem_helper_t<>::attach(usb);
                return true;
            }
            __INLINE bool detach()
            {
                auto& usb = *static_cast<USB*>(this);
                elem_helper_t<>::detach(usb);
                state = STATE_DETACHED;
                return true;
            }
            __INLINE bool reset()
            {
                if (state == STATE_DETACHED)
                    return false;
                state = STATE_DEFAULT;
                auto& usb = *static_cast<USB*>(this);
                elem_helper_t<>::reset(usb);
                return true;
            }
            __INLINE bool suspend()
            {
                if (state == STATE_DETACHED)
                    return false;
                auto& usb = *static_cast<USB*>(this);
                elem_helper_t<>::suspend(usb);
                state |= STATE_FLAG_SUSPENDED;
                return true;
            }
            __INLINE bool resume()
            {
                if (state == STATE_DETACHED)
                    return false;
                state &= ~STATE_FLAG_SUSPENDED;
                auto& usb = *static_cast<USB*>(this);
                elem_helper_t<>::resume(usb);
                return true;
            }

            __INLINE void setup(uint_fast8_t type, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                assert(state & STATE_FLAG_DEFAULT);
                auto& usb = *static_cast<USB*>(this);
                switch (type) {
                case request_type_t::DEVICE_STANDARD_OUT:
                    if (usb.standard_request_out(request, value, index, length))
                        return;
                    break;
                case request_type_t::INTERFACE_STANDARD_OUT:
                    assert(state == STATE_CONFIGURED);
                    if (helper_t<>::if_standard_request_out(usb, request, value, index >> 8, length, index & 0xff))
                        return;
                    break;
                case request_type_t::ENDPOINT_STANDARD_OUT:
                    assert(state == STATE_CONFIGURED);
                    assert(index != 0 && index != 0x80);
                    if (helper_t<>::ep_standard_request_out(usb, request, value, (index & 0xf) - 1, length))
                        return;
                    break;
                case request_type_t::DEVICE_CLASS_OUT:
                    assert(state == STATE_CONFIGURED);
                    if (usb.class_request_out(request, value, index, length))
                        return;
                    break;
                case request_type_t::INTERFACE_CLASS_OUT:
                    assert(state == STATE_CONFIGURED);
                    if (helper_t<>::if_class_request_out(usb, request, value, index >> 8, length, index & 0xff))
                        return;
                    break;
                case request_type_t::ENDPOINT_CLASS_OUT:
                    assert(state == STATE_CONFIGURED);
                    assert(index != 0 && index != 0x80);
                    if (helper_t<>::ep_class_request_out(usb, request, value, (index & 0xf) - 1, length))
                        return;
                    break;
                case request_type_t::DEVICE_VENDOR_OUT:
                    assert(state == STATE_CONFIGURED);
                    if (usb.vendor_request_out(request, value, index, length))
                        return;
                    break;
                case request_type_t::INTERFACE_VENDOR_OUT:
                    assert(state == STATE_CONFIGURED);
                    if (helper_t<>::if_vendor_request_out(usb, request, value, index >> 8, length, index & 0xff))
                        return;
                    break;
                case request_type_t::ENDPOINT_VENDOR_OUT:
                    assert(state == STATE_CONFIGURED);
                    assert(index != 0 && index != 0x80);
                    if (helper_t<>::ep_vendor_request_out(usb, request, value, (index & 0xf) - 1, length))
                        return;
                    break;
                case request_type_t::DEVICE_STANDARD_IN:
                    assert(state == STATE_CONFIGURED);
                    if (usb.standard_request_in(request, value, index, length))
                        return;
                    break;
                case request_type_t::INTERFACE_STANDARD_IN:
                    assert(state == STATE_CONFIGURED);
                    if (helper_t<>::if_standard_request_in(usb, request, value, index >> 8, length, index & 0xff))
                        return;
                    break;
                case request_type_t::ENDPOINT_STANDARD_IN:
                    assert(state == STATE_CONFIGURED);
                    assert(index != 0 && index != 0x80);
                    if (helper_t<>::ep_standard_request_in(usb, request, value, (index & 0xf) - 1, length))
                        return;
                    break;
                case request_type_t::DEVICE_CLASS_IN:
                    assert(state == STATE_CONFIGURED);
                    if (usb.class_request_in(request, value, index, length))
                        return;
                    break;
                case request_type_t::INTERFACE_CLASS_IN:
                    assert(state == STATE_CONFIGURED);
                    if (helper_t<>::if_class_request_in(usb, request, value, index >> 8, length, index & 0xff))
                        return;
                    break;
                case request_type_t::ENDPOINT_CLASS_IN:
                    assert(state == STATE_CONFIGURED);
                    assert(index != 0 && index != 0x80);
                    if (helper_t<>::ep_class_request_in(usb, request, value, (index & 0xf) - 1, length))
                        return;
                    break;
                case request_type_t::DEVICE_VENDOR_IN:
                    assert(state == STATE_CONFIGURED);
                    if (usb.vendor_request_in(request, value, index, length))
                        return;
                    break;
                case request_type_t::INTERFACE_VENDOR_IN:
                    assert(state == STATE_CONFIGURED);
                    if (helper_t<>::if_vendor_request_in(usb, request, value, index >> 8, length, index & 0xff))
                        return;
                    break;
                case request_type_t::ENDPOINT_VENDOR_IN:
                    assert(state == STATE_CONFIGURED);
                    assert(index != 0 && index != 0x80);
                    if (helper_t<>::ep_vendor_request_in(usb, request, value, (index & 0xf) - 1, length))
                        return;
                    break;
                default:
                    assert(false);
                }
                ep0_t::set_stall();
            }
            __INLINE void transact_in(uint_fast8_t type, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                if ((type & 0x80) == 0) {
                    if (usb.status_in(type, request, value, index, length))
                        return;
                } else {
                    if (usb.data_in(type, request, value, index, length))
                        return;
                }
                ep0_t::set_stall();
            }
            __INLINE bool status_in(uint_fast8_t type, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                switch (type) {
                case request_type_t::DEVICE_STANDARD_OUT:
                    return usb.standard_status_in(request, value, index, length);
                case request_type_t::INTERFACE_STANDARD_OUT:
                    return helper_t<>::if_standard_status_in(usb, request, value, index >> 8, length, index & 0xff);
                case request_type_t::ENDPOINT_STANDARD_OUT:
                    assert(index != 0 && index != 0x80);
                    return helper_t<>::ep_standard_status_in(usb, request, value, (index & 0xf) - 1, length);
                case request_type_t::DEVICE_CLASS_OUT:
                    return usb.class_status_in(request, value, index, length);
                case request_type_t::INTERFACE_CLASS_OUT:
                    return helper_t<>::if_class_status_in(usb, request, value, index >> 8, length, index & 0xff);
                case request_type_t::ENDPOINT_CLASS_OUT:
                    assert(index != 0 && index != 0x80);
                    return helper_t<>::ep_class_status_in(usb, request, value, (index & 0xf) - 1, length);
                case request_type_t::DEVICE_VENDOR_OUT:
                    return usb.vendor_status_in(request, value, index, length);
                case request_type_t::INTERFACE_VENDOR_OUT:
                    return helper_t<>::if_vendor_status_in(usb, request, value, index >> 8, length, index & 0xff);
                case request_type_t::ENDPOINT_VENDOR_OUT:
                    assert(index != 0 && index != 0x80);
                    return helper_t<>::ep_vendor_status_in(usb, request, value, (index & 0xf) - 1, length);
                default:
                    assert(false);
                    return true;
                }
            }
            __INLINE bool data_in(uint_fast8_t type, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                switch (type) {
                case request_type_t::DEVICE_STANDARD_IN:
                    return usb.standard_data_in(request, value, index, length);
                case request_type_t::INTERFACE_STANDARD_IN:
                    return helper_t<>::if_standard_data_in(usb, request, value, index >> 8, length, index & 0xff);
                case request_type_t::ENDPOINT_STANDARD_IN:
                    assert(index != 0 && index != 0x80);
                    return helper_t<>::ep_standard_data_in(usb, request, value, (index & 0xf) - 1, length);
                case request_type_t::DEVICE_CLASS_IN:
                    return usb.class_data_in(request, value, index, length);
                case request_type_t::INTERFACE_CLASS_IN:
                    return helper_t<>::if_class_data_in(usb, request, value, index >> 8, length, index & 0xff);
                case request_type_t::ENDPOINT_CLASS_IN:
                    assert(index != 0 && index != 0x80);
                    return helper_t<>::ep_class_data_in(usb, request, value, (index & 0xf) - 1, length);
                case request_type_t::DEVICE_VENDOR_IN:
                    return usb.vendor_data_in(request, value, index, length);
                case request_type_t::INTERFACE_VENDOR_IN:
                    return helper_t<>::if_vendor_data_in(usb, request, value, index >> 8, length, index & 0xff);
                case request_type_t::ENDPOINT_VENDOR_IN:
                    assert(index != 0 && index != 0x80);
                    return helper_t<>::ep_vendor_data_in(usb, request, value, (index & 0xf) - 1, length);
                default:
                    assert(false);
                    in();
                    return true;
                }
            }
            __INLINE void transact_out(uint_fast8_t type, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                if ((type & 0x80) == 0) {
                    if (usb.data_out(type, request, value, index, length))
                        return;
                } else {
                    if (usb.status_out(type, request, value, index, length))
                        return;
                }
                ep0_t::set_stall();
            }
            __INLINE bool data_out(uint_fast8_t type, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                switch (type) {
                case request_type_t::DEVICE_STANDARD_OUT:
                    return usb.standard_data_out(request, value, index, length);
                case request_type_t::INTERFACE_STANDARD_OUT:
                    return helper_t<>::if_standard_data_out(usb, request, value, index >> 8, length, index & 0xff);
                case request_type_t::ENDPOINT_STANDARD_OUT:
                    assert(index != 0 && index != 0x80);
                    return helper_t<>::ep_standard_data_out(usb, request, value, (index & 0xf) - 1, length);
                case request_type_t::DEVICE_CLASS_OUT:
                    return usb.class_data_out(request, value, index, length);
                case request_type_t::INTERFACE_CLASS_OUT:
                    return helper_t<>::if_class_data_out(usb, request, value, index >> 8, length, index & 0xff);
                case request_type_t::ENDPOINT_CLASS_OUT:
                    assert(index != 0 && index != 0x80);
                    return helper_t<>::ep_class_data_out(usb, request, value, (index & 0xf) - 1, length);
                case request_type_t::DEVICE_VENDOR_OUT:
                    return usb.vendor_data_out(request, value, index, length);
                case request_type_t::INTERFACE_VENDOR_OUT:
                    return helper_t<>::if_vendor_data_out(usb, request, value, index >> 8, length, index & 0xff);
                case request_type_t::ENDPOINT_VENDOR_OUT:
                    assert(index != 0 && index != 0x80);
                    return helper_t<>::ep_vendor_data_out(usb, request, value, (index & 0xf) - 1, length);
                default:
                    assert(false);
                    out();
                    return true;
                }
            }
            __INLINE bool status_out(uint_fast8_t type, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                switch (type) {
                case request_type_t::DEVICE_STANDARD_IN:
                    return usb.standard_status_out(request, value, index, length);
                case request_type_t::INTERFACE_STANDARD_IN:
                    return helper_t<>::if_standard_status_out(usb, request, value, index >> 8, length, index & 0xff);
                case request_type_t::ENDPOINT_STANDARD_IN:
                    assert(index != 0 && index != 0x80);
                    return helper_t<>::ep_standard_status_out(usb, request, value, (index & 0xf) - 1, length);
                case request_type_t::DEVICE_CLASS_IN:
                    return usb.class_status_out(request, value, index, length);
                case request_type_t::INTERFACE_CLASS_IN:
                    return helper_t<>::if_class_status_out(usb, request, value, index >> 8, length, index & 0xff);
                case request_type_t::ENDPOINT_CLASS_IN:
                    assert(index != 0 && index != 0x80);
                    return helper_t<>::ep_class_status_out(usb, request, value, (index & 0xf) - 1, length);
                case request_type_t::DEVICE_VENDOR_IN:
                    return usb.vendor_status_out(request, value, index, length);
                case request_type_t::INTERFACE_VENDOR_IN:
                    return helper_t<>::if_vendor_status_out(usb, request, value, index >> 8, length, index & 0xff);
                case request_type_t::ENDPOINT_VENDOR_IN:
                    assert(index != 0 && index != 0x80);
                    return helper_t<>::ep_vendor_status_out(usb, request, value, (index & 0xf) - 1, length);
                default:
                    assert(false);
                    return true;
                }
            }
            __INLINE bool standard_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                switch (request_t(request)) {
                case request_t::GET_STATUS:
                    assert(state != STATE_DEFAULT);
                    assert(length == sizeof(uint16_t));
                    if (state == STATE_CONFIGURED)
                        return usb.get_status_setup(length);
                    return false;
                case request_t::GET_DESCRIPTOR:
                    assert(state & (STATE_FLAG_DEFAULT | STATE_FLAG_ADDRESS | STATE_FLAG_CONFIGURED));
                    assert(length != 0);
                    return usb.get_descriptor_setup(value, index, length);
                case request_t::GET_CONFIGURATION:
                    assert(state != STATE_DEFAULT);
                    assert(value == 0);
                    assert(index == 0);
                    assert(length == sizeof(uint8_t));
                    return usb.get_configuration_setup(length);
                default:
                    assert(false);
                    return false;
                }
            }
            __INLINE bool standard_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                switch (request_t(request)) {
                case request_t::CLEAR_FEATURE:
                    assert(state == STATE_ADDRESS || state == STATE_CONFIGURED);
                    assert(index == 0);
                    assert(length == 0);
                    return usb.clear_feature_setup(value);
                case request_t::SET_FEATURE:
                    assert(state == STATE_ADDRESS || state == STATE_CONFIGURED);
                    assert(index == 0);
                    assert(length == 0);
                    return usb.set_feature_setup(value);
                case request_t::SET_ADDRESS:
                    assert(state != STATE_CONFIGURED);
                    assert(index == 0);
                    assert(length == 0);
                    return usb.set_address_setup(value);
                case request_t::SET_DESCRIPTOR:
                    assert(state & (STATE_FLAG_ADDRESS | STATE_FLAG_CONFIGURED));
                    assert(length != 0);
                    return usb.set_descriptor_setup(value, index, length);
                case request_t::SET_CONFIGURATION:
                    assert(state == STATE_ADDRESS);
                    assert(value != 0);
                    assert(index == 0);
                    assert(length == 0);
                    return usb.set_configuration_setup(value);
                default:
                    assert(false);
                    return false;
                }
            }
            __INLINE bool standard_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                switch (request_t(request)) {
                case request_t::GET_STATUS:
                    return usb.get_status_data(length);
                case request_t::GET_DESCRIPTOR:
                    return usb.get_descriptor_data(value, index, length);
                case request_t::GET_CONFIGURATION:
                    return usb.get_configuration_data(length);
                default:
                    assert(false);
                    in();
                    return true;
                }
            }
            __INLINE bool standard_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                switch (request_t(request)) {
                case request_t::SET_DESCRIPTOR:
                    return usb.set_descriptor_data(value, index, length);
                default:
                    assert(false);
                    out();
                    return true;
                }
            }
            __INLINE bool standard_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                switch (request_t(request)) {
                case request_t::CLEAR_FEATURE:
                    return usb.clear_feature_status(value);
                case request_t::SET_FEATURE:
                    return usb.set_feature_status(value);
                case request_t::SET_ADDRESS: {
                    uint_fast8_t address = value;
                    if (usb.set_address_status(address)) {
                        if (address != 0)
                            state |= STATE_FLAG_ADDRESS;
                        else
                            state = STATE_DEFAULT;
                        return true;
                    }
                    return false;
                }
                case request_t::SET_DESCRIPTOR:
                    return usb.set_descriptor_status(value, index, length);
                case request_t::SET_CONFIGURATION:
                    if (usb.set_configuration_status(value)) {
                        state |= STATE_FLAG_CONFIGURED;
                        return true;
                    }
                    return false;
                default:
                    assert(false);
                    return true;
                }
            }
            __INLINE bool standard_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                switch (request_t(request)) {
                case request_t::GET_STATUS:
                    return usb.get_status_status(length);
                case request_t::GET_DESCRIPTOR:
                    return usb.get_descriptor_status(value, index, length);
                case request_t::GET_CONFIGURATION:
                    return usb.get_configuration_status(length);
                default:
                    assert(false);
                    return true;
                }
            }
            __INLINE bool class_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
            __INLINE bool class_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
            __INLINE bool class_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                in();
                return true;
            }
            __INLINE bool class_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                out();
                return true;
            }
            __INLINE bool class_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
            __INLINE bool class_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
            __INLINE bool vendor_request_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
            __INLINE bool vendor_request_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
            __INLINE bool vendor_data_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                usb.in();
                return true;
            }
            __INLINE bool vendor_data_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
            {
                out();
                return true;
            }
            __INLINE bool vendor_status_in(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
            __INLINE bool vendor_status_out(uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
            __INLINE bool get_descriptor_setup(uint_fast16_t value, uint_fast16_t lang_id, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                uint_fast8_t desc_index = value & 0xff;
                switch (value >> 8) {
                case DESC_DEVICE:
                    return usb.get_device_descriptor_setup(length);
                case DESC_CONFIG:
                    return usb.get_config_descriptor_setup(length);
                case DESC_STRING:
                    return usb.get_string_descriptor_setup(desc_index, lang_id, length);
                default:
                    assert(false);
                    return false;
                }
            }
            __INLINE bool get_descriptor_data(uint_fast16_t value, uint_fast16_t lang_id, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                uint_fast8_t desc_index = value & 0xff;
                switch (value >> 8) {
                case DESC_DEVICE:
                    return usb.get_device_descriptor_data(length);
                case DESC_CONFIG:
                    return usb.get_config_descriptor_data(length);
                case DESC_STRING:
                    return usb.get_string_descriptor_data(desc_index, lang_id, length);
                default:
                    assert(false);
                    in();
                    return true;
                }
            }
            __INLINE bool get_descriptor_status(uint_fast16_t value, uint_fast16_t lang_id, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                uint_fast8_t desc_index = value & 0xff;
                switch (value >> 8) {
                case DESC_DEVICE:
                    return usb.get_device_descriptor_status(length);
                case DESC_CONFIG:
                    return usb.get_config_descriptor_status(length);
                case DESC_STRING:
                    return usb.get_string_descriptor_status(desc_index, lang_id, length);
                default:
                    assert(false);
                    return true;
                }
            }
            __INLINE bool set_descriptor_setup(uint_fast16_t value, uint_fast16_t lang_id, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                uint_fast8_t desc_index{value & 0xff};
                switch (value >> 8) {
                case DESC_DEVICE:
                    assert(desc_index == 0);
                    assert(lang_id == 0);
                    return usb.set_device_descriptor_setup(length);
                case DESC_CONFIG:
                    assert(desc_index == 0);
                    assert(lang_id == 0);
                    return usb.set_config_descriptor_setup(length);
                case DESC_STRING:
                    return usb.set_string_descriptor_setup(desc_index, lang_id, length);
                default:
                    assert(false);
                    return false;
                }
            }
            __INLINE bool set_descriptor_data(uint_fast16_t value, uint_fast16_t lang_id, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                uint_fast8_t desc_index = value & 0xff;
                switch (value >> 8) {
                case DESC_DEVICE:
                    assert(desc_index == 0);
                    assert(lang_id == 0);
                    return usb.set_device_descriptor_data(length);
                case DESC_CONFIG:
                    assert(desc_index == 0);
                    assert(lang_id == 0);
                    return usb.set_config_descriptor_data(length);
                case DESC_STRING:
                    return usb.set_string_descriptor_data(desc_index, lang_id, length);
                default:
                    assert(false);
                    out();
                    return true;
                }
            }
            __INLINE bool set_descriptor_status(uint_fast16_t value, uint_fast16_t lang_id, uint_fast16_t length)
            {
                auto& usb = *static_cast<USB*>(this);
                uint_fast8_t desc_index = value & 0xff;
                switch (value >> 8) {
                case DESC_DEVICE:
                    assert(desc_index == 0);
                    assert(lang_id == 0);
                    return usb.set_device_descriptor_status(length);
                case DESC_CONFIG:
                    assert(desc_index == 0);
                    assert(lang_id == 0);
                    return usb.set_config_descriptor_status(length);
                case DESC_STRING:
                    return usb.set_string_descriptor_status(desc_index, lang_id, length);
                default:
                    assert(false);
                    return true;
                }
            }
            __INLINE bool get_status_setup(uint_fast16_t length)
            {
                const void* buffer = static_cast<USB*>(this)->get_status(length);
                if (buffer != nullptr) {
                    in(buffer, length, length);
                    return true;
                }
                return false;
            }
            __INLINE bool get_status_data(uint_fast16_t length)
            {
                in();
                return true;
            }
            __INLINE bool get_status_status(uint_fast16_t length) { return true; }
            __INLINE bool clear_feature_setup(uint_fast8_t selector)
            {
                if (static_cast<USB*>(this)->clear_feature(selector)) {
                    out(nullptr, 0);
                    return true;
                }
                return false;
            }
            __INLINE bool clear_feature_status(uint_fast8_t selector) { return true; }
            __INLINE bool set_feature_setup(uint_fast8_t selector)
            {
                if (static_cast<USB*>(this)->set_feature(selector)) {
                    out(nullptr, 0);
                    return true;
                }
                return false;
            }
            __INLINE bool set_feature_status(uint_fast8_t selector) { return true; }
            __INLINE bool set_address_setup(uint_fast8_t address)
            {
                out(nullptr, 0);
                return true;
            }
            __INLINE bool set_address_status(uint_fast8_t address)
            {
                return static_cast<USB*>(this)->set_address(address);
            }
            __INLINE bool get_device_descriptor_setup(uint_fast16_t length)
            {
                auto& descriptor = static_cast<USB*>(this)->get_device_descriptor();
                in(&descriptor, sizeof(descriptor), length);
                return true;
            }
            __INLINE bool get_device_descriptor_data(uint_fast16_t length)
            {
                in();
                return true;
            }
            __INLINE bool get_device_descriptor_status(uint_fast16_t length) { return true; }
            __INLINE bool get_config_descriptor_setup(uint_fast16_t length)
            {
                auto& descriptor = static_cast<USB*>(this)->get_config_descriptor();
                in(&descriptor, sizeof(descriptor), length);
                return true;
            }
            __INLINE bool get_config_descriptor_data(uint_fast16_t length)
            {
                in();
                return true;
            }
            __INLINE bool get_config_descriptor_status(uint_fast16_t length) { return true; }
            __INLINE bool get_string_descriptor_setup(uint_fast8_t desc_index, uint_fast16_t lang_id, uint_fast16_t length)
            {
                const uint8_t* p = static_cast<USB*>(this)->get_string_descriptor(desc_index, lang_id);
                in(p, *p, length);
                return true;
            }
            __INLINE bool get_string_descriptor_data(uint_fast8_t desc_index, uint_fast16_t lang_id, uint_fast16_t length)
            {
                in();
                return true;
            }
            __INLINE bool get_string_descriptor_status(uint_fast8_t desc_index, uint_fast16_t lang_id, uint_fast16_t length) { return true; }
            __INLINE bool set_device_descriptor_setup(uint_fast16_t length)
            {
                void* descriptor = static_cast<USB*>(this)->set_device_descriptor(length);
                if (descriptor != nullptr) {
                    out(descriptor, length);
                    return true;
                }
                return false;
            }
            __INLINE bool set_device_descriptor_data(uint_fast16_t length)
            {
                out();
                return true;
            }
            __INLINE bool set_device_descriptor_status(uint_fast16_t length) { return true; }
            __INLINE bool set_config_descriptor_setup(uint_fast16_t length)
            {
                void* descriptor = static_cast<USB*>(this)->set_config_descriptor(length);
                if (descriptor != nullptr) {
                    out(descriptor, length);
                    return true;
                }
                return false;
            }
            __INLINE bool set_config_descriptor_data(uint_fast16_t length)
            {
                out();
                return true;
            }
            __INLINE bool set_config_descriptor_status(uint_fast16_t length) { return true; }
            __INLINE bool set_string_descriptor_setup(uint_fast8_t desc_index, uint_fast16_t lang_id, uint_fast16_t length)
            {
                void* descriptor = static_cast<USB*>(this)->set_string_descriptor(desc_index, lang_id, length);
                if (descriptor != nullptr) {
                    out(descriptor, length);
                    return true;
                }
                return false;
            }
            __INLINE bool set_string_descriptor_data(uint_fast8_t desc_index, uint_fast16_t lang_id, uint_fast16_t length)
            {
                out();
                return true;
            }
            __INLINE bool set_string_descriptor_status(uint_fast8_t desc_index, uint_fast16_t lang_id, uint_fast16_t length) { return true; }
            __INLINE bool get_configuration_setup(uint_fast16_t length)
            {
                const void* buffer = static_cast<USB*>(this)->get_configuration(length);
                if (buffer != nullptr) {
                    in(buffer, length, length);
                    return true;
                }
                return false;
            }
            __INLINE bool get_configuration_data(uint_fast16_t length)
            {
                in();
                return true;
            }
            __INLINE bool get_configuration_status(uint_fast16_t length) { return true; }
            __INLINE bool set_configuration_setup(uint_fast8_t configuration)
            {
                if (static_cast<USB*>(this)->set_configuration(configuration)) {
                    out(nullptr, 0);
                    return true;
                }
                return false;
            }
            __INLINE bool set_configuration_status(uint_fast8_t configuration) { return true; }
            __INLINE const void* get_status(uint_fast16_t length) { return nullptr; }
            __INLINE bool clear_feature(uint_fast8_t selector) { return false; }
            __INLINE bool set_feature(uint_fast8_t selector) { return false; }
            __INLINE const device_descriptor_t& get_device_descriptor()
            {
                static const device_descriptor_t descriptor __attribute__((aligned(4)));
                return descriptor;
            }
            __INLINE const hierarchical_descriptor_t& get_config_descriptor()
            {
                static const hierarchical_descriptor_t descriptor __attribute__((aligned(4)));
                return descriptor;
            }
            __PURE const uint8_t* get_string_descriptor(uint_fast8_t index, uint_fast16_t lang_id);
            __INLINE void* set_device_descriptor(uint_fast16_t length) { return nullptr; }
            __INLINE void* set_config_descriptor(uint_fast16_t length) { return nullptr; }
            __INLINE void* set_string_descriptor(uint_fast8_t desc_index, uint_fast16_t lang_id, uint_fast16_t length) { return nullptr; }
            __INLINE bool set_configuration(uint_fast8_t configuration)
            {
                elem_helper_t<>::config(*static_cast<USB*>(this));
                return true;
            }
            __INLINE const uint8_t* get_configuration(uint_fast16_t length) { return nullptr; }

            __INLINE void ep_event(uint_fast8_t index)
            {
                auto& usb = *static_cast<USB*>(this);
                if (index == 0)
                    usb.event();
                else
                    elem_helper_t<>::event(usb, index - 1);
            }

        private:
            template<typename T = void, uint8_t N = 0>
            struct helper_t {
                __INLINE static bool if_standard_request_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).standard_request_out(request, value, index, length) : helper_t<T, N + 1>::if_standard_request_out(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_standard_request_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_standard_request_out(request, value, index, length) : helper_t<T, N + 1>::ep_standard_request_out(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_standard_request_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).standard_request_in(request, value, index, length) : helper_t<T, N + 1>::if_standard_request_in(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_standard_request_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_standard_request_in(request, value, index, length) : helper_t<T, N + 1>::ep_standard_request_in(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_class_request_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).class_request_out(request, value, index, length) : helper_t<T, N + 1>::if_class_request_out(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_class_request_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_class_request_out(request, value, index, length) : helper_t<T, N + 1>::ep_class_request_out(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_class_request_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).class_request_in(request, value, index, length) : helper_t<T, N + 1>::if_class_request_in(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_class_request_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_class_request_in(request, value, index, length) : helper_t<T, N + 1>::ep_class_request_in(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_vendor_request_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).vendor_request_out(request, value, index, length) : helper_t<T, N + 1>::if_vendor_request_out(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_vendor_request_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_vendor_request_out(request, value, index, length) : helper_t<T, N + 1>::ep_vendor_request_out(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_vendor_request_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).vendor_request_in(request, value, index, length) : helper_t<T, N + 1>::if_vendor_request_in(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_vendor_request_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_vendor_request_in(request, value, index, length) : helper_t<T, N + 1>::ep_vendor_request_in(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_standard_data_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).standard_data_out(request, value, index, length) : helper_t<T, N + 1>::if_standard_data_out(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_standard_data_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_standard_data_out(request, value, index, length) : helper_t<T, N + 1>::ep_standard_data_out(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_standard_data_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).standard_data_in(request, value, index, length) : helper_t<T, N + 1>::if_standard_data_in(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_standard_data_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_standard_data_in(request, value, index, length) : helper_t<T, N + 1>::ep_standard_data_in(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_class_data_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).class_data_out(request, value, index, length) : helper_t<T, N + 1>::if_class_data_out(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_class_data_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_class_data_out(request, value, index, length) : helper_t<T, N + 1>::ep_class_data_out(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_class_data_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).class_data_in(request, value, index, length) : helper_t<T, N + 1>::if_class_data_in(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_class_data_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_class_data_in(request, value, index, length) : helper_t<T, N + 1>::ep_class_data_in(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_vendor_data_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).vendor_data_out(request, value, index, length) : helper_t<T, N + 1>::if_vendor_data_out(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_vendor_data_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_vendor_data_out(request, value, index, length) : helper_t<T, N + 1>::ep_vendor_data_out(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_vendor_data_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).vendor_data_in(request, value, index, length) : helper_t<T, N + 1>::if_vendor_data_in(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_vendor_data_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_vendor_data_in(request, value, index, length) : helper_t<T, N + 1>::ep_vendor_data_in(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_standard_status_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).standard_status_out(request, value, index, length) : helper_t<T, N + 1>::if_standard_status_out(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_standard_status_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_standard_status_out(request, value, index, length) : helper_t<T, N + 1>::ep_standard_status_out(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_standard_status_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).standard_status_in(request, value, index, length) : helper_t<T, N + 1>::if_standard_status_in(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_standard_status_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_standard_status_in(request, value, index, length) : helper_t<T, N + 1>::ep_standard_status_in(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_class_status_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).class_status_out(request, value, index, length) : helper_t<T, N + 1>::if_class_status_out(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_class_status_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_class_status_out(request, value, index, length) : helper_t<T, N + 1>::ep_class_status_out(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_class_status_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).class_status_in(request, value, index, length) : helper_t<T, N + 1>::if_class_status_in(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_class_status_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_class_status_in(request, value, index, length) : helper_t<T, N + 1>::ep_class_status_in(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_vendor_status_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).vendor_status_out(request, value, index, length) : helper_t<T, N + 1>::if_vendor_status_out(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_vendor_status_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_vendor_status_out(request, value, index, length) : helper_t<T, N + 1>::ep_vendor_status_out(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
                __INLINE static bool if_vendor_status_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    return if_index == N ? get_if<N>(usb).vendor_status_in(request, value, index, length) : helper_t<T, N + 1>::if_vendor_status_in(usb, request, value, index, length, if_index);
                }
                __INLINE static bool ep_vendor_status_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    return index < get_if<N>(usb)._ep_count ? get_if<N>(usb).ep_vendor_status_in(request, value, index, length) : helper_t<T, N + 1>::ep_vendor_status_in(usb, request, value, index - get_if<N>(usb)._ep_count, length);
                }
            };

            template<typename T>
            struct helper_t<T, USB::_if_count> {
                __INLINE static bool if_standard_request_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index) { return false; }
                __INLINE static bool ep_standard_request_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
                __INLINE static bool if_standard_request_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index) { return false; }
                __INLINE static bool ep_standard_request_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
                __INLINE static bool if_class_request_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index) { return false; }
                __INLINE static bool ep_class_request_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
                __INLINE static bool if_class_request_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index) { return false; }
                __INLINE static bool ep_class_request_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
                __INLINE static bool if_vendor_request_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index) { return false; }
                __INLINE static bool ep_vendor_request_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
                __INLINE static bool if_vendor_request_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index) { return false; }
                __INLINE static bool ep_vendor_request_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return false; }
                __INLINE static bool if_standard_data_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    usb.out();
                    return true;
                }
                __INLINE static bool ep_standard_data_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    usb.out();
                    return true;
                }
                __INLINE static bool if_standard_data_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    usb.in();
                    return true;
                }
                __INLINE static bool ep_standard_data_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    usb.in();
                    return true;
                }
                __INLINE static bool if_class_data_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    usb.out();
                    return true;
                }
                __INLINE static bool ep_class_data_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    usb.out();
                    return true;
                }
                __INLINE static bool if_class_data_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    usb.in();
                    return true;
                }
                __INLINE static bool ep_class_data_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    usb.in();
                    return true;
                }
                __INLINE static bool if_vendor_data_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    usb.out();
                    return true;
                }
                __INLINE static bool ep_vendor_data_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    usb.out();
                    return true;
                }
                __INLINE static bool if_vendor_data_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index)
                {
                    usb.in();
                    return true;
                }
                __INLINE static bool ep_vendor_data_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length)
                {
                    usb.in();
                    return true;
                }
                __INLINE static bool if_standard_status_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index) { return true; }
                __INLINE static bool ep_standard_status_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool if_standard_status_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index) { return true; }
                __INLINE static bool ep_standard_status_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool if_class_status_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index) { return true; }
                __INLINE static bool ep_class_status_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool if_class_status_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index) { return true; }
                __INLINE static bool ep_class_status_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool if_vendor_status_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index) { return true; }
                __INLINE static bool ep_vendor_status_out(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
                __INLINE static bool if_vendor_status_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast8_t index, uint_fast16_t length, uint_fast8_t if_index) { return true; }
                __INLINE static bool ep_vendor_status_in(USB& usb, uint_fast8_t request, uint_fast16_t value, uint_fast16_t index, uint_fast16_t length) { return true; }
            };

            template<typename T = void, uint8_t N = 0>
            struct elem_helper_t {
                __INLINE static void attach(USB& usb)
                {
                    get_elem<N>(usb).attach();
                    elem_helper_t<T, N + 1>::attach(usb);
                }
                __INLINE static void detach(USB& usb)
                {
                    get_elem<N>(usb).detach();
                    elem_helper_t<T, N + 1>::detach(usb);
                }
                __INLINE static void reset(USB& usb)
                {
                    get_elem<N>(usb).reset();
                    elem_helper_t<T, N + 1>::reset(usb);
                }
                __INLINE static void suspend(USB& usb)
                {
                    get_elem<N>(usb).suspend();
                    elem_helper_t<T, N + 1>::suspend(usb);
                }
                __INLINE static void resume(USB& usb)
                {
                    get_elem<N>(usb).resume();
                    elem_helper_t<T, N + 1>::resume(usb);
                }
                __INLINE static void config(USB& usb)
                {
                    get_elem<N>(usb).config();
                    elem_helper_t<T, N + 1>::config(usb);
                }
                __INLINE static void event(USB& usb, uint_fast8_t index)
                {
                    if (index < get_elem<N>(usb)._ep_count)
                        get_elem<N>(usb).event(index);
                    elem_helper_t<T, N + 1>::event(usb, index - get_elem<N>(usb)._ep_count);
                }
            };

            template<typename T>
            struct elem_helper_t<T, USB::_elem_count> {
                __INLINE static void attach(USB& usb) { }
                __INLINE static void detach(USB& usb) { }
                __INLINE static void reset(USB& usb) { }
                __INLINE static void suspend(USB& usb) { }
                __INLINE static void resume(USB& usb) { }
                __INLINE static void config(USB& usb) { }
                __INLINE static void event(USB& usb, uint_fast8_t index) { }
            };

            uint8_t state;
        };
    }

    template<langid_t... IDS>
    struct __attribute__((aligned(4), packed)) string_langid_t {
        uint8_t bLength{ sizeof(wLANGID) + 2 };
        uint8_t bDescriptorType{ core::DESC_STRING };
        langid_t wLANGID[sizeof...(IDS)]{ IDS... };
    };

    template<char16_t CH, char16_t... STRING>
    struct __attribute__((aligned(4), packed)) string_t {
        uint8_t bLength{ sizeof(bString) + 2 };
        uint8_t bDescriptorType{ core::DESC_STRING };
        char16_t bString[sizeof...(STRING) + 1]{ CH, STRING... };
    };
}

#endif  // __USB_CORE_HPP
