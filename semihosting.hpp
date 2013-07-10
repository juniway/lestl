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

#ifndef __LESTL_SEMIHOSTING_HPP
#define __LESTL_SEMIHOSTING_HPP

#ifndef __arm__
#error "Only ARM target is supported"
#else

#include <cstdint>
#include <attribute.h>

namespace lestl {
    namespace semihosting {
        enum operation_t {
            SYS_OPEN = 1,
            SYS_WRITE = 5,
            SYS_READ = 6,
        };

        __INLINE static intptr_t call0(operation_t op, const void* arg1 = 0)
        {
            register const void* p asm ("a2") = arg1;
            __asm__ __volatile__("" ::: "memory");
            register uintptr_t c asm ("a1") = op;
            register uintptr_t r asm ("r0");
#ifdef __THUMBEL__
            __asm__ __volatile__("bkpt #0xab" : "=r" (r) : "0" (c), "r" (p));
#else
            __asm__ __volatile__("swi #0x123456" : "=r" (r) : "0" (c), "r" (p));
#endif
            return r;
        }

        template<typename T1>
        __INLINE static intptr_t call(operation_t op, T1 arg1)
        {
            uintptr_t block[1] = { uintptr_t(arg1) };
            return call0(op, block);
        }

        template<typename T1, typename T2>
        __INLINE static intptr_t call(operation_t op, T1 arg1, T2 arg2)
        {
            uintptr_t block[2] = { uintptr_t(arg1), uintptr_t(arg2) };
            return call0(op, block);
        }

        template<typename T1, typename T2, typename T3>
        __INLINE static intptr_t call(operation_t op, T1 arg1, T2 arg2, T3 arg3)
        {
            uintptr_t block[3] = { uintptr_t(arg1), uintptr_t(arg2), uintptr_t(arg3) };
            return call0(op, block);
        }

        template<typename T1, typename T2, typename T3, typename T4>
        __INLINE static intptr_t call(operation_t op, T1 arg1, T2 arg2, T3 arg3, T4 arg4)
        {
            uintptr_t block[4] = { uintptr_t(arg1), uintptr_t(arg2), uintptr_t(arg3), uintptr_t(arg4) };
            return call0(op, block);
        }

        class console_t {
        public:
            __INLINE console_t()
            {
                handle = call(SYS_OPEN, ":tt", 3, 3);
            }

            __INLINE int write(const void* str, uintptr_t len)
            {
                return call(SYS_WRITE, handle, str, len);
            }

            __INLINE int read(void* str, uintptr_t len)
            {
                return call(SYS_READ, handle, str, len);
            }

            uintptr_t handle;
        };
    }
}
#endif  // __arm__

#endif  // __LESTL_SEMIHOSTING_HPP
