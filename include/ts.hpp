/*
 * Convenience header for c++ projects.
 * If the library is compiled with a c compiler but used in a c++ project, use this header instead.
 */

#ifndef TS_HPP_
#define TS_HPP_

extern "C" {
#include "ts.h"
}

#endif  // TS_HPP_
