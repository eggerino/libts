#include "ts.h"
#include "ts_sys.h"

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

ts_vec3* ts_vec3_scale(const ts_vec3* v, ts_f64 factor, ts_vec3* result) {
    result->x = v->x * factor;
    result->y = v->y * factor;
    result->z = v->z * factor;
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

ts_f64 ts_vec3_norm_l2(const ts_vec3* v) {
    ts_f64 norm_sqr = ts_vec3_dot(v, v);
    return ts_sqrt(norm_sqr);
}

ts_vec3* ts_vec3_normalize(const ts_vec3* v, ts_vec3* result) {
    ts_f64 norm_sqr = ts_vec3_dot(v, v);
    if (norm_sqr < TS_EPS) {
        result->x = 1.0;
        result->y = 0.0;
        result->z = 0.0;
        return result;
    }

    ts_f64 norm = ts_sqrt(norm_sqr);
    ts_f64 factor = 1.0 / norm;  // Safe because zero checked
    return ts_vec3_scale(v, factor, result);
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

ts_quat* ts_quat_scale(const ts_quat* q, ts_f64 factor, ts_quat* result) {
    result->w = q->w * factor;
    result->x = q->x * factor;
    result->y = q->y * factor;
    result->z = q->z * factor;
    return result;
}

ts_f64 ts_quat_dot(const ts_quat* a, const ts_quat* b) { return a->w * b->w + a->x * b->x + a->y * b->y + a->z * b->z; }

ts_quat* ts_quat_mul(const ts_quat* a, const ts_quat* b, ts_quat* result) {
    ts_f64 w, x, y, z;
    w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->y;

    result->w = w;
    result->x = x;
    result->y = y;
    result->z = z;
    return result;
}

ts_quat* ts_quat_normalize(const ts_quat* q, ts_quat* result) {
    ts_f64 norm_sqr = ts_quat_dot(q, q);
    if (norm_sqr < TS_EPS) {
        result->w = 1.0;
        result->x = 0.0;
        result->y = 0.0;
        result->z = 0.0;
        return result;
    }

    ts_f64 norm = ts_sqrt(norm_sqr);
    ts_f64 factor = 1.0 / norm;
    return ts_quat_scale(q, factor, result);
}

ts_mat3x3* ts_mat3x3_add(const ts_mat3x3* a, const ts_mat3x3* b, ts_mat3x3* result) {
    result->xx = a->xx + b->xx;
    result->xy = a->xy + b->xy;
    result->xz = a->xz + b->xz;

    result->yx = a->yx + b->yx;
    result->yy = a->yy + b->yy;
    result->yz = a->yz + b->yz;

    result->zx = a->zx + b->zx;
    result->zy = a->zy + b->zy;
    result->zz = a->zz + b->zz;
    return result;
}

ts_mat3x3* ts_mat3x3_sub(const ts_mat3x3* a, const ts_mat3x3* b, ts_mat3x3* result) {
    result->xx = a->xx - b->xx;
    result->xy = a->xy - b->xy;
    result->xz = a->xz - b->xz;

    result->yx = a->yx - b->yx;
    result->yy = a->yy - b->yy;
    result->yz = a->yz - b->yz;

    result->zx = a->zx - b->zx;
    result->zy = a->zy - b->zy;
    result->zz = a->zz - b->zz;
    return result;
}

ts_mat3x3* ts_mat3x3_mul(const ts_mat3x3* a, const ts_mat3x3* b, ts_mat3x3* result) {
    ts_f64 xx, xy, xz, yx, yy, yz, zx, zy, zz;
    xx = a->xx * b->xx + a->xy * b->yx + a->xz * b->zx;
    xy = a->xx * b->xy + a->xy * b->yy + a->xz * b->zy;
    xz = a->xx * b->xz + a->xy * b->yz + a->xz * b->zz;

    yx = a->yx * b->xx + a->yy * b->yx + a->yz * b->zx;
    yy = a->yx * b->xy + a->yy * b->yy + a->yz * b->zy;
    yz = a->yx * b->xz + a->yy * b->yz + a->yz * b->zz;

    zx = a->zx * b->xx + a->zy * b->yx + a->zz * b->zx;
    zy = a->zx * b->xy + a->zy * b->yy + a->zz * b->zy;
    zz = a->zx * b->xz + a->zy * b->yz + a->zz * b->zz;

    result->xx = xx;
    result->xy = xy;
    result->xz = xz;
    result->yy = yy;
    result->yz = yz;
    result->zz = zz;
    return result;
}

ts_vec3* ts_mat3x3_vec3_mul(const ts_mat3x3* a, const ts_vec3* b, ts_vec3* result) {
    ts_f64 x, y, z;
    x = a->xx * b->x + a->xy * b->y + a->xz * b->z;
    y = a->yx * b->x + a->yy * b->y + a->yz * b->z;
    z = a->zx * b->x + a->zy * b->y + a->zz * b->z;

    result->x = x;
    result->y = y;
    result->z = z;
    return result;
}

ts_quat* ts_mat3x3_to_quat(const ts_mat3x3* m, ts_quat* result) {
    ts_f32 t, factor;
    if (m->zz < 0) {
        if (m->xx > m->yy) {
            t = 1 + m->xx - m->yy - m->zz;
            result->w = t;
            result->x = m->xy + m->yx;
            result->y = m->zx + m->xz;
            result->z = m->yz - m->zy;
        } else {
            t = 1 - m->xx + m->yy - m->zz;
            result->w = m->xy + m->yx;
            result->x = t;
            result->y = m->yz + m->zy;
            result->z = m->zx - m->xz;
        }
    } else {
        if (m->xx < -m->yy) {
            t = 1 - m->xx - m->yy + m->zz;
            result->w = m->zx + m->xz;
            result->x = m->yz + m->zy;
            result->y = t;
            result->z = m->xy - m->yx;
        } else {
            t = 1 + m->xx + m->yy + m->zz;
            result->w = m->yz - m->zy;
            result->x = m->zx - m->xz;
            result->y = m->xy - m->yx;
            result->z = t;
        }
    }
    factor = 0.5 / ts_sqrt(t);
    return ts_quat_scale(result, factor, result);
}

ts_mat3x3* ts_quat_to_mat3x3(const ts_quat* q, ts_mat3x3* result) {
    result->xx = 1.0 - 2.0 * (q->y * q->y + q->z * q->z);
    result->xy = 2.0 * (-q->w * q->z + q->x * q->y);
    result->xz = 2.0 * (q->w * q->y + q->x * q->z);

    result->xy = 2.0 * (q->w * q->z + q->x * q->y);
    result->yy = 1.0 - 2.0 * (q->x * q->x + q->z * q->z);
    result->yz = 2.0 * (-q->w * q->x + q->y * q->z);

    result->zx = 2.0 * (-q->w * q->y + q->x * q->z);
    result->zy = 2.0 * (q->w * q->x + q->y * q->z);
    result->zz = 1.0 - 2.0 * (q->x * q->x + q->y * q->y);
    return result;
}
