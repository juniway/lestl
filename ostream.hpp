/*
  Copyright (c) 2011-2013  John Lee (j.y.lee@yeah.net)
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

#ifndef __LESTL_OSTREAM_HPP
#define __LESTL_OSTREAM_HPP

#include <cstdint>
#include <cstddef>
#include <attribute.h>

namespace lestl {
    struct setfill_t { char fill; };
    struct setw_t { int width; };
    struct hex_t { unsigned long data; };
    struct fdec_t { long long data; };

    template<typename FILE, uint8_t BUFSZ>
    class ostream_t : FILE {
    public:
        __INLINE ostream_t() : data{ ' ' } { }
        __NOINLINE ostream_t& put(const char* str, size_t len)
        {
            size_t w = width;
            width = 0;
            while (len < w) {
                put(fill);
                --w;
            }
            do
                put(*str++);
            while (--len);
            return *this;
        }

        __INLINE ostream_t& put(char c)
        {
            buf[nchar++] = c;
            if (nchar == BUFSZ)
                flush();
            return *this;
        }

        __INLINE ostream_t& flush()
        {
            this->write(buf, nchar);
            nchar = 0;
            return *this;
        }

        __INLINE ostream_t& setfill(char c)
        {
            fill = c;
            return *this;
        }

        __INLINE ostream_t& setw(int w)
        {
            width = w;
            return *this;
        }

        __INLINE ostream_t& operator <<(ostream_t& (*pf)(ostream_t&))
        {
            return pf(*this);
        }

        union {
            struct {
                char fill;
                uint8_t nchar;
                uint8_t width;
            };
            uint32_t data;
        };
        char __attribute__((aligned(4))) buf[BUFSZ];
    };

    template<typename T>
    class ostream_t<T, 0> {
    public:
        __INLINE constexpr ostream_t() { }
        __INLINE ostream_t& put(const char* str, size_t len) { return *this; }
        __INLINE ostream_t& put(char c) { return *this; }
        __INLINE ostream_t& flush() { return *this; }
        __INLINE ostream_t& setfill(char c) { return *this; }
        __INLINE ostream_t& setw(int w) { return *this; }
        __INLINE ostream_t& operator <<(ostream_t& (*pf)(ostream_t&)) { return *this; }
    };

    __WEAK char* l10toa(char *p, unsigned long val);
    __WEAK char* f10toa(char *p, long val, uint_fast8_t point);
    __INLINE hex_t hex(unsigned long val);
    __INLINE setfill_t setfill(char c);
    __INLINE setw_t setw(int w);
    __INLINE fdec_t fdec(long val, uint_fast8_t pt);

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, char c)
    {
        return os.put(&c, 1);
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, signed char c)
    {
        return os << static_cast<char>(c);
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, unsigned char c)
    {
        return os << static_cast<char>(c);
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, bool b)
    {
        return os << (b ? '1' : '0');
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, const void* p)
    {
        return os << hex(long(p));
    }

    template<typename FILE, uint8_t BUFSZ>
    __WEAK ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, const char* str)
    {
        return os.put(str, __builtin_strlen(str));
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, const signed char* str)
    {
        return os << reinterpret_cast<const char*>(str);
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, const unsigned char* str)
    {
        return os << reinterpret_cast<const char*>(str);
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, int val)
    {
        return os << static_cast<long>(val);
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, unsigned int val)
    {
        return os << static_cast<unsigned long>(val);
    }

    template<typename FILE, uint8_t BUFSZ>
    __WEAK ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, unsigned long val)
    {
        char buf[12];
        char* p = l10toa(&buf[12], val);
        return os.put(p, &buf[12] - p);
    }

    template<typename FILE, uint8_t BUFSZ>
    __WEAK ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, long val)
    {
        char buf[12];
        bool b = val < 0;
        if (b)
            val = -val;
        char* p = l10toa(&buf[12], val);
        if (b)
            *--p = '-';
        return os.put(p, &buf[12] - p);
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& ends(ostream_t<FILE, BUFSZ>& os)
    {
        return os.put('\0');
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& flush(ostream_t<FILE, BUFSZ>& os)
    {
        return os.flush();
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& endl(ostream_t<FILE, BUFSZ>& os)
    {
        return flush(os.put('\n'));
    }

    __INLINE setfill_t setfill(char c)
    {
        return { c };
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, setfill_t f)
    {
        return os.setfill(f.fill);
    }

    __INLINE setw_t setw(int w)
    {
        return { w };
    }

    template<typename FILE, uint8_t BUFSZ>
    __INLINE ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, setw_t f)
    {
        return os.setw(f.width);
    }

    __INLINE hex_t hex(unsigned long val)
    {
        return { val };
    }

    __INLINE fdec_t fdec(long val, uint_fast8_t pt)
    {
        union {
            long long ll;
            struct {
                long l;
                long pt;
            };
        } v;
        v.l = val;
        v.pt = pt;
        return { v.ll };
    }

    template<typename FILE, uint8_t BUFSZ>
    __WEAK ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, hex_t h)
    {
        char buf[8];
        char* p = &buf[8];

        unsigned long val = h.data;
        do {
            uint_fast8_t ch = val & 0xf;
            ch += ch > 9 ? 'A' - 10 : '0';
            *--p = ch;
            val >>= 4;
        } while (val != 0);
        return os.put(p, &buf[8] - p);
    }

    template<typename FILE, uint8_t BUFSZ>
    __WEAK ostream_t<FILE, BUFSZ>& operator <<(ostream_t<FILE, BUFSZ>& os, fdec_t d)
    {
        char buf[12];
        char* p = f10toa(&buf[12], d.data, d.data >> 32);
        return os.put(p, &buf[12] - p);
    }

    __WEAK char* l10toa(char *p, unsigned long val)
    {
        do {
            auto quot = val / 10;
            *--p = val - quot * 10 + '0';
            val = quot;
        } while (val != 0);
        return p;
    }

    __WEAK char* f10toa(char *p, long val, uint_fast8_t point)
    {
        int n = point;
        if (val < 0) {
            val = -val;
            n |= 0x80;
        }
        do {
            auto quot = val / 10;
            auto rem = val - quot * 10;
            if (n < 0 || rem != 0) {
                n |= 1 << 31;
                *--p = rem + '0';
            }
            val = quot;
        } while ((--n & 0x1f) != 0);
        if (n < 0)
            *--p = '.';
        p = l10toa(p, val);
        if (n & 0x80)
            *--p = '-';
        return p;
    }
}

#endif  // __LESTL_OSTREAM_HPP
