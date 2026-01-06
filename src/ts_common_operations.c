#include "ts.h"

ts_none ts_vec3_add(const ts_vec3* a, const ts_vec3* b, ts_vec3* result) {
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;
}

ts_none ts_vec3_sub(const ts_vec3* a, const ts_vec3* b, ts_vec3* result) {
    result->x = a->x - b->x;
    result->y = a->y - b->y;
    result->z = a->z - b->z;
}

ts_f64 ts_quat_dot(const ts_quat* a, const ts_quat* b) { return a->w * b->w + a->x * b->x + a->y * b->y + a->z * b->z; }
