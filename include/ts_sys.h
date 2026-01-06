/*
 * Abstraction over the standard library.
 * By default the types are used from the standard library.
 * There is an implementation file in src/ts_sys_libc.c which implements required functions with libc.
 *
 * If you cannot use libc, you can provide your own implementations and link with them instead.
 * You can also provide your own header ts_sys_types.h which defines the used primitive types.
 * Compile the with the TS_NO_STDLIB_TYPES directive and your own types are used instead.
 */

#ifndef TS_SYS_H_
#define TS_SYS_H_

// Primitive types
#ifdef TS_NO_STDLIB_TYPES

// Use the custom types
#include "ts_sys_types.h"

#else

#include <stddef.h>
#include <stdint.h>

// None or any types
typedef void ts_none;
typedef void* ts_ptr;

// unsigned integers
typedef uint8_t ts_u8;
typedef uint16_t ts_u16;
typedef uint32_t ts_u32;
typedef uint64_t ts_u64;
typedef size_t ts_usize;

// signed integers
typedef int8_t ts_i8;
typedef int16_t ts_i16;
typedef int32_t ts_i32;
typedef int64_t ts_i64;

// Floating point numbers
typedef float ts_f32;
typedef double ts_f64;

#endif

// Dynamic memory management
ts_ptr ts_mem_alloc(ts_usize size);
ts_none ts_mem_free(ts_ptr ptr);

// Math
ts_f64 ts_sqrt(ts_f64 x);
ts_f64 ts_sin(ts_f64 x);
ts_f64 ts_cos(ts_f64 x);
ts_f64 ts_acos(ts_f64 x);

#endif  // TS_SYS_H_
