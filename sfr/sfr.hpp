#ifndef __SFR_HPP
#define __SFR_HPP

#include <cstdint>
#include <attribute.h>

namespace sfr {
    template<typename SFR, uintptr_t ADDR>
    __INLINE static void set_addr(SFR& sfr)
    {
        __asm__ __volatile__(
            ".global %c[sym]"   "\n\t"
            ".weak %c[sym]"     "\n\t"
            ".set %c[sym],%c[addr]"
            : : [sym] "i" (&sfr), [addr] "i" (ADDR)
        );
    }

    template<typename R, uint8_t S, uint8_t W>
    struct sfb_t {                      // Special Function Bit
        enum {
            LSB = S,
            WIDTH = W,
            MASK = (((1LL << W) - 1) << S)
        };
        __INLINE sfb_t() {}

        __INLINE uintptr_t lsb() { return LSB; }
        __INLINE uintptr_t lsb() const { return LSB; }
        __INLINE uintptr_t width() { return WIDTH; }
        __INLINE uintptr_t width() const { return WIDTH; }
        __INLINE typename R::type mask() { return MASK; }
        __INLINE typename R::type mask() const { return MASK; }
        __INLINE typename R::type read() { return (reinterpret_cast<R*>(this)->val & MASK) >> LSB; }
        __INLINE typename R::type read() const { return (reinterpret_cast<const R*>(this)->val & MASK) >> LSB; }
        __INLINE operator typename R::type() { return read(); }
        __INLINE operator typename R::type() const { return read(); }

        __INLINE R& read(typename R::type& v)
        {
            v = read();
            return *reinterpret_cast<R*>(this);
        }

        __INLINE R& read(typename R::type& v) const
        {
            v = read();
            return *reinterpret_cast<R*>(this);
        }

        __INLINE R& write(typename R::type v, bool safe = true)
        {
            register R* p = reinterpret_cast<R*>(this);
            typename R::type _v = v;
            p->changed |= MASK;
            _v <<= LSB;
            if (!__builtin_constant_p(v) || ((~_v & MASK) != 0 && (p->zero_mask & MASK) != 0))
                p->val &= ~MASK;
            if (safe)
                _v &= MASK;
            p->val |= _v;
            p->zero_mask |= __builtin_constant_p(v) ? _v : (~0 & MASK);
            return *p;
        }

        __INLINE R& operator ()(typename R::type v, bool safe = true) { return write(v, safe); }

        __INLINE R& operator =(typename R::type v) { return write(v, true).apply(); }

        template<typename T>
        __INLINE R& lambda(T v)
        {
            register R* p = reinterpret_cast<R*>(this);
            register typename R::type data = (p->val & MASK) >> LSB;
            register auto res = v(data);
            if (data != res)
                write(res);
            return *p;
        }

        template<typename T>
        __INLINE R& lambda(T v) const
        {
            register R* p = reinterpret_cast<R*>(this);
            register typename R::type data = (p->val & MASK) >> LSB;
            register auto res = v(data);
            return *p;
        }

        __INLINE R& operator &=(typename R::type v)
        {
            register R* p = reinterpret_cast<R*>(this);
            register typename R::type data = (v << LSB) | ~MASK;
            if (data != typename R::type(~0)) {
                p->changed |= MASK;
                p->val &= data;
            }
            return *p;
        }

        __INLINE R& operator ^=(typename R::type v)
        {
            register R* p = reinterpret_cast<R*>(this);
            if (v != 0) {
                p->changed |= MASK;
                p->val ^= (v << LSB) & MASK;
            }
            return *p;
        }

        __INLINE R& rol(uint_fast8_t n)
        {
            register R* p = reinterpret_cast<R*>(this);
            p->changed |= MASK;
            register typename R::type v = p->val & MASK;
            p->val &= ~MASK;
            p->val |= ((v << n) | (v >> W - n)) & MASK;
            return *p;
        }

        __INLINE R& ror(uint_fast8_t n)
        {
            return rol(W - n);
        }
    };

    template<typename SFR>
    class sfr_t {                       // Special Function Register
    public:
        __INLINE constexpr sfr_t() { }
        __INLINE sfr_t(typename SFR::type v) : val(v) { }
        __INLINE operator typename SFR::type() { return val; }
        __INLINE operator typename SFR::type() const { return val; }
        __INLINE operator typename SFR::type() volatile { return val; }
        __INLINE operator typename SFR::type() const volatile { return val; }

        __INLINE sfr_t<SFR> operator =(SFR v)
        {
            val = v.val;
            return *this;
        }

        __INLINE sfr_t<SFR> operator =(SFR v) volatile
        {
            val = v.val;
            return *this;
        }

        __INLINE typename SFR::type operator =(typename SFR::type v)
        {
            val = v;
            return v;
        }

        __INLINE typename SFR::type operator =(typename SFR::type v) volatile
        {
            val = v;
            return v;
        }

        __INLINE SFR operator ()() { return SFR(val); }
        __INLINE SFR operator ()() const { return SFR(val); }
        __INLINE SFR operator ()() volatile { return SFR(val); }
        __INLINE SFR operator ()() const volatile { return SFR(val); }
        __INLINE SFR operator ()(typename SFR::type v) { return SFR(val, v); }
        __INLINE SFR operator ()(typename SFR::type v) volatile { return SFR(val, v); }

        volatile typename SFR::type val;
    };
}

#define SFR_ADDR(SFR, ADDR) __SFR_ADDR(__COUNTER__, SFR, ADDR)
#define __SFR_ADDR(CNT, SFR, ADDR)                                          \
    __attribute__((naked, used)) static void __CONCAT(__sfr_addr, CNT)()    \
    {                                                                       \
        sfr::set_addr<decltype(SFR), ADDR>(SFR);                            \
    }

#define __SFR(SFR, TYPE, WRITE)                                             \
    typedef TYPE type;                                                      \
    __INLINE SFR(type volatile& r) : ref(r), val(r), changed(0), zero_mask(~0) { }                                          \
    __INLINE SFR(type const volatile& r) : ref(const_cast<type volatile&>(r)), val(r), changed(0), zero_mask(~0) { }        \
    __INLINE SFR(type volatile& r, type v) : ref(r), val(v), changed(~0), zero_mask(__builtin_constant_p(v) ? v : ~0) {  }  \
    __INLINE ~SFR()                                                         \
    {                                                                       \
        if (changed != 0)                                                   \
            apply();                                                        \
    }                                                                       \
    template<uint8_t S, uint8_t W>                                          \
    __INLINE sfr::sfb_t<SFR, S, W>& field()                                 \
    {                                                                       \
        return *reinterpret_cast<sfr::sfb_t<SFR, S, W>*>(this);             \
    }                                                                       \
    __INLINE SFR& apply()                                                   \
    {                                                                       \
        if ((WRITE & 1) != 0 && (changed & 0xffffff00) == 0)                \
            reinterpret_cast<volatile uint8_t*>(&ref)[0] = val;             \
        else if ((WRITE & 1) != 0 && (changed & 0xffff00ff) == 0)           \
            reinterpret_cast<volatile uint8_t*>(&ref)[1] = val >> 8;        \
        else if ((WRITE & 1) != 0 && (changed & 0xff00ffff) == 0)           \
            reinterpret_cast<volatile uint8_t*>(&ref)[2] = val >> 16;       \
        else if ((WRITE & 1) != 0 && (changed & 0x00ffffff) == 0)           \
            reinterpret_cast<volatile uint8_t*>(&ref)[3] = val >> 24;       \
        else if ((WRITE & 2) != 0 && (changed & 0xffff0000) == 0)           \
            reinterpret_cast<volatile uint16_t*>(&ref)[0] = val;            \
        else if ((WRITE & 2) != 0 && (changed & 0xffff) == 0)               \
            reinterpret_cast<volatile uint16_t*>(&ref)[1] = val >> 16;      \
        else                                                                \
            ref = val;                                                      \
        __asm__ __volatile__("" ::: "memory");                              \
        changed = 0;                                                        \
        return *this;                                                       \
    }                                                                       \
    struct {                                                                \
        type volatile& ref;                                                 \
        type val;                                                           \
        type changed;                                                       \
        type zero_mask;                                                     \
    };

#endif  // __SFR_HPP
