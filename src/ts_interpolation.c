#include "ts.h"
#include "ts_sys.h"

ts_f64 ts_interpolate_linear1(ts_f64 start, ts_f64 end, ts_f64 x) { return (end - start) * x + start; }

ts_f64 ts_interpolate_linear1_d(ts_f64 start, ts_f64 end) { return end - start; }

ts_f64 ts_interpolate_linear1_dd() { return 0; }

ts_vec3* ts_interpolate_linear3(const ts_vec3* start, const ts_vec3* end, ts_f64 x, ts_vec3* result) {
    result->x = ts_interpolate_linear1(start->x, end->x, x);
    result->y = ts_interpolate_linear1(start->y, end->y, x);
    result->z = ts_interpolate_linear1(start->z, end->z, x);
    return result;
}

ts_vec3* ts_interpolate_linear3_d(const ts_vec3* start, const ts_vec3* end, ts_vec3* result) {
    return ts_vec3_sub(end, start, result);
}

ts_vec3* ts_interpolate_linear3_dd(ts_vec3* result) {
    result->x = 0;
    result->y = 0;
    result->z = 0;
    return result;
}

ts_quat* ts_interpolate_slerp(const ts_quat* start, const ts_quat* end, ts_f64 x, ts_quat* result) {
    ts_f64 cos_omega = ts_quat_dot(start, end);
    ts_f64 omega = ts_acos(cos_omega);
    ts_f64 sin_omega = ts_sin(omega);
    ts_f64 one_over_sin_omega = 1.0 / sin_omega;

    ts_f64 x_start = ts_sin((1 - x) * omega) * one_over_sin_omega;
    ts_f64 x_end = ts_sin(x * omega) * one_over_sin_omega;

    result->w = x_start * start->w + x_end * end->w;
    result->x = x_start * start->x + x_end * end->x;
    result->y = x_start * start->y + x_end * end->y;
    result->z = x_start * start->z + x_end * end->z;
    return result;
}

ts_quat* ts_interpolate_slerp_d(const ts_quat* start, const ts_quat* end, ts_f64 x, ts_f64 dx, ts_quat* result) {
    ts_f64 cos_omega = ts_quat_dot(start, end);
    ts_f64 omega = ts_acos(cos_omega);
    ts_f64 sin_omega = ts_sin(omega);
    ts_f64 one_over_sin_omega = 1.0 / sin_omega;

    ts_f64 dx_start = -ts_cos((1 - x) * omega) * one_over_sin_omega * omega * dx;
    ts_f64 dx_end = ts_cos(x * omega) * one_over_sin_omega * omega * dx;

    result->w = dx_start * start->w + dx_end * end->w;
    result->x = dx_start * start->x + dx_end * end->x;
    result->y = dx_start * start->y + dx_end * end->y;
    result->z = dx_start * start->z + dx_end * end->z;
    return result;
}

ts_quat* ts_interpolate_slerp_dd(const ts_quat* start, const ts_quat* end, ts_f64 x, ts_f64 dx, ts_f64 ddx,
                                 ts_quat* result) {
    ts_f64 cos_omega = ts_quat_dot(start, end);
    ts_f64 omega = ts_acos(cos_omega);
    ts_f64 sin_omega = ts_sin(omega);
    ts_f64 one_over_sin_omega = 1.0 / sin_omega;

    ts_f64 ddx_start = -ts_cos((1 - x) * omega) * one_over_sin_omega * omega * ddx -
                       ts_sin((1 - x) * omega) * one_over_sin_omega * omega * omega * dx * dx;
    ts_f64 ddx_end = ts_cos(x * omega) * one_over_sin_omega * omega * ddx -
                     ts_sin(x * omega) * one_over_sin_omega * omega * omega * dx * dx;

    result->w = ddx_start * start->w + ddx_end * end->w;
    result->x = ddx_start * start->x + ddx_end * end->x;
    result->y = ddx_start * start->y + ddx_end * end->y;
    result->z = ddx_start * start->z + ddx_end * end->z;
    return result;
}
