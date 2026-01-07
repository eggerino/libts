#include "ts.h"
#include "ts_sys.h"

static inline ts_vec3* vec3_copy(const ts_vec3* src, ts_vec3* dest) {
    dest->x = src->x;
    dest->y = src->y;
    dest->z = src->z;
    return dest;
}

static inline ts_quat* quat_copy(const ts_quat* src, ts_quat* dest) {
    dest->w = src->w;
    dest->x = src->x;
    dest->y = src->y;
    dest->z = src->z;
    return dest;
}

ts_vec3* ts_vec3_scale(const ts_vec3* v, ts_f64 factor, ts_vec3* result) {
    result->x = v->x * factor;
    result->y = v->y * factor;
    result->z = v->z * factor;
    return result;
}

ts_f64 ts_vec3_norm_l2(const ts_vec3* v) {
    ts_f64 norm_sqr = ts_vec3_dot(v, v);
    return ts_sqrt(norm_sqr);
}

ts_vec3* ts_vec3_normalize(const ts_vec3* v, ts_vec3* result) {
    ts_f64 norm_sqr = ts_vec3_dot(v, v);
    if (norm_sqr < TS_EPS) {
        return vec3_copy(v, result);
    }

    ts_f64 norm = ts_sqrt(norm_sqr);
    ts_f64 factor = 1.0 / norm;  // Safe because zero checked
    return ts_vec3_scale(v, factor, result);
}

ts_vec3* ts_vec3_add(const ts_vec3* a, const ts_vec3* b, ts_vec3* result) {
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;
    return result;
}

ts_vec3* ts_vec3_sub(const ts_vec3* a, const ts_vec3* b, ts_vec3* result) {
    result->x = a->x - b->x;
    result->y = a->y - b->y;
    result->z = a->z - b->z;
    return result;
}

ts_f64 ts_vec3_dot(const ts_vec3* a, const ts_vec3* b) { return a->x * b->x + a->y * b->y + a->z * b->z; }

ts_vec3* ts_vec3_cross(const ts_vec3* a, const ts_vec3* b, ts_vec3* result) {
    ts_f64 x, y, z;
    x = a->y * b->z - a->z * b->y;
    y = a->z * b->x - a->x * b->z;
    z = a->x * b->y - a->y * b->x;

    result->x = x;
    result->y = y;
    result->z = z;
    return result;
}

ts_quat* ts_quat_scale(const ts_quat* q, ts_f64 factor, ts_quat* result) {
    result->w = q->w * factor;
    result->x = q->x * factor;
    result->y = q->y * factor;
    result->z = q->z * factor;
    return result;
}

ts_quat* ts_quat_normalize(const ts_quat* q, ts_quat* result) {
    ts_f64 norm_sqr = ts_quat_dot(q, q);
    if (norm_sqr < TS_EPS) {
        return quat_copy(q, result);
    }

    ts_f64 norm = ts_sqrt(norm_sqr);
    ts_f64 factor = 1.0 / norm;
    return ts_quat_scale(q, factor, result);
}

ts_quat* ts_quat_add(const ts_quat* a, const ts_quat* b, ts_quat* result) {
    result->w = a->w + b->w;
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;
    return result;
}

ts_quat* ts_quat_sub(const ts_quat* a, const ts_quat* b, ts_quat* result) {
    result->w = a->w - b->w;
    result->x = a->x - b->x;
    result->y = a->y - b->y;
    result->z = a->z - b->z;
    return result;
}

ts_f64 ts_quat_dot(const ts_quat* a, const ts_quat* b) { return a->w * b->w + a->x * b->x + a->y * b->y + a->z * b->z; }
