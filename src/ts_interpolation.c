#include "ts.h"

ts_f64 ts_interpolate_linear1(ts_f64 start, ts_f64 end, ts_f64 x) { return (end - start) * x + start; }

ts_none ts_interpolate_linear3(const ts_vec3* start, const ts_vec3* end, ts_f64 x, ts_vec3* result) {
    result->x = ts_interpolate_linear1(start->x, end->x, x);
    result->y = ts_interpolate_linear1(start->y, end->y, x);
    result->z = ts_interpolate_linear1(start->z, end->z, x);
}
