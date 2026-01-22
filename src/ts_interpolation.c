#include "ts.h"
#include "ts_sys.h"

#define vec3_copy(src, dest) \
    (dest).x = (src).x;      \
    (dest).y = (src).y;      \
    (dest).z = (src).z

#define quat_copy(src, dest)  \
    vec3_copy((src), (dest)); \
    (dest).w = (src).w

ts_linear1_state* ts_linear1_init(ts_f64 start, ts_f64 end, ts_linear1_state* inst) {
    inst->start = start;
    inst->delta = end - start;
    return inst;
}

ts_f64 ts_linear1(const ts_linear1_state* inst, ts_f64 x) { return inst->delta * x + inst->start; }

ts_f64 ts_linear1_d(const ts_linear1_state* inst) { return inst->delta; }

ts_f64 ts_linear1_dd(ts_none) { return 0; }

ts_linear3_state* ts_linear3_init(const ts_vec3* start, const ts_vec3* end, ts_linear3_state* inst) {
    vec3_copy(*start, inst->start);
    ts_vec3_sub(end, start, &inst->delta);
    return inst;
}

ts_vec3* ts_linear3(const ts_linear3_state* inst, ts_f64 x, ts_vec3* result) {
    return ts_vec3_add(&inst->start, ts_vec3_scale(&inst->delta, x, result), result);
}

ts_vec3* ts_linear3_d(const ts_linear3_state* inst, ts_vec3* result) {
    vec3_copy(inst->delta, *result);
    return result;
}

ts_vec3* ts_linear3_dd(ts_vec3* result) {
    result->x = 0;
    result->y = 0;
    result->z = 0;
    return result;
}

ts_slerp_state* ts_slerp_init(const ts_quat* start, const ts_quat* end, ts_slerp_state* inst) {
    quat_copy(*start, inst->start);
    quat_copy(*end, inst->end);
    ts_f64 cos_omega = ts_quat_dot(start, end);
    inst->omega = ts_acos(cos_omega);
    ts_f64 sin_omega = ts_sin(inst->omega);
    inst->one_over_sin_omega = 1.0 / sin_omega;
    return inst;
}

ts_quat* ts_slerp(const ts_slerp_state* inst, ts_f64 x, ts_quat* result) {
    ts_f64 x_start = ts_sin((1 - x) * inst->omega) * inst->one_over_sin_omega;
    ts_f64 x_end = ts_sin(x * inst->omega) * inst->one_over_sin_omega;
    
    result->w = x_start * inst->start.w + x_end * inst->end.w;
    result->x = x_start * inst->start.x + x_end * inst->end.x;
    result->y = x_start * inst->start.y + x_end * inst->end.y;
    result->z = x_start * inst->start.z + x_end * inst->end.z;
    return result;
}

ts_quat* ts_slerp_d(const ts_slerp_state* inst, ts_f64 x, ts_quat* result) {
    ts_f64 dx_start = -ts_cos((1 - x) * inst->omega) * inst->one_over_sin_omega * inst->omega;
    ts_f64 dx_end = ts_cos(x * inst->omega) * inst->one_over_sin_omega * inst->omega;
    
    result->w = dx_start * inst->start.w + dx_end * inst->end.w;
    result->x = dx_start * inst->start.x + dx_end * inst->end.x;
    result->y = dx_start * inst->start.y + dx_end * inst->end.y;
    result->z = dx_start * inst->start.z + dx_end * inst->end.z;
    return result;
}

ts_quat* ts_slerp_dd(const ts_slerp_state* inst, ts_f64 x, ts_quat* result) {
    ts_slerp(inst, x, result);
    return ts_quat_scale(result, -inst->omega * inst->omega, result);
}
