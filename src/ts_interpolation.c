#include "ts.h"
#include "ts_sys.h"

static ts_f64 quat_dot(const ts_quat* a, const ts_quat* b) { return a->w * b->w + a->x * b->x + a->y * b->y + a->z * b->z; }

ts_f64 ts_interpolate_linear1(ts_f64 start, ts_f64 end, ts_f64 x) { return (end - start) * x + start; }

ts_none ts_interpolate_linear3(const ts_vec3* start, const ts_vec3* end, ts_f64 x, ts_vec3* result) {
    result->x = ts_interpolate_linear1(start->x, end->x, x);
    result->y = ts_interpolate_linear1(start->y, end->y, x);
    result->z = ts_interpolate_linear1(start->z, end->z, x);
}

ts_none ts_interpolate_slerp(const ts_quat* start, const ts_quat* end, ts_f64 x, ts_quat* result) {
    ts_f64 cos_omega = quat_dot(start, end);
    ts_f64 omega = ts_acos(cos_omega);
    ts_f64 sin_omega = ts_sin(omega);
    ts_f64 one_over_sin_omega = 1.0 / sin_omega;

    ts_f64 x_start = ts_sin((1 - x) * omega) * one_over_sin_omega;
    ts_f64 x_end = ts_sin(x * omega) * one_over_sin_omega;

    result->w = x_start * start->w + x_end * end->w;
    result->x = x_start * start->x + x_end * end->x;
    result->y = x_start * start->y + x_end * end->y;
    result->z = x_start * start->z + x_end * end->z;
}
