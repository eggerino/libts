#define _USE_MATH_DEFINES

#include <math.h>
#include <stdlib.h>

#include "ts_sys.h"

ts_ptr ts_mem_alloc(ts_usize size) { return malloc(size); }
ts_none ts_mem_free(ts_ptr ptr) { free(ptr); }
ts_f64 ts_sqrt(ts_f64 x) { return sqrt(x); }
ts_f64 ts_sin(ts_f64 x) { return sin(x); }
ts_f64 ts_cos(ts_f64 x) { return cos(x); }
