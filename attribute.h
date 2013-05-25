#ifndef __ATTRIBUTE_H
#define __ATTRIBUTE_H

#ifndef __PURE
#define __PURE      __attribute__((error("pure virtual mothed called")))
#endif  // __PURE

#ifndef __INLINE
#define __INLINE    __attribute__((always_inline)) inline
#endif  // __INLINE

#ifndef __NOINLINE
#define __NOINLINE  __attribute__((noinline))
#endif  // __NOINLINE

#ifndef __WEAK
#define __WEAK  __attribute__((noinline, weak))
#endif  // __NOINLINE

#ifndef __CONCAT
#define __CONCAT(a, b)      __CONCAT_I(a, b)
#define __CONCAT_I(a, b)    a ## b
#endif  // __CONCAT

#endif  // __ATTRIBUTE_H
