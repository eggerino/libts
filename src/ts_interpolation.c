#include "ts.h"
#include "ts_sys.h"

#define vec3_copy(src, dest) \
    (dest).x = (src).x;      \
    (dest).y = (src).y;      \
    (dest).z = (src).z

#define vec3_zero(v) \
    (v).x = 0;       \
    (v).y = 0;       \
    (v).z = 0

#define quat_copy(src, dest) \
    (dest).w = (src).w;      \
    vec3_copy((src), (dest))

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

ts_allocation_error ts_bezier_init(const ts_vec3* points, ts_usize num_points, ts_bezier_state* inst) {
    /*
     * The buffer needs to hold at most 2 pages of values, first and second derivatives.
     * in order for the interpolation alogorithms to have enough memory available.
     */

    inst->points = ts_mem_alloc(num_points * sizeof(*(inst->points)));
    if (!inst->points) {
        return TS_ALLOC_ERR;
    }

    ts_usize buffer_size = 2 * 3 * num_points * sizeof(*(inst->points));
    inst->buffer = ts_mem_alloc(buffer_size);
    if (!inst->buffer) {
        ts_mem_free(inst->points);
        return TS_ALLOC_ERR;
    }

    inst->num_points = num_points;
    ts_usize i;
    for (i = 0; i < num_points; ++i) {
        vec3_copy(points[i], inst->points[i]);
    }

    return TS_ALLOC_OK;
}

ts_none ts_bezier_deinit(ts_bezier_state* inst) {
    ts_mem_free(inst->points);
    ts_mem_free(inst->buffer);
}

#define BEZIER_SEGMENT(page) &(((ts_vec3*)(inst->buffer))[(page) * inst->num_points])

#define BEZIER_SWAP_SEGMENT(a, b) \
    temp = a;                     \
    a = b;                        \
    b = temp

#define BEZIER_COMPUTE_NEXT_VALUE                          \
    ts_vec3_scale(&(prev_value[idx]), one_minus_x, &left); \
    ts_vec3_scale(&(prev_value[idx + 1]), x, &right);      \
    ts_vec3_add(&left, &right, &(cur_value[idx]))

#define BEZIER_COMPUTE_NEXT_FIRST_DER                          \
    ts_vec3_scale(&(prev_first_der[idx]), one_minus_x, &left); \
    ts_vec3_sub(&left, &(prev_value[idx]), &left);             \
    ts_vec3_scale(&(prev_first_der[idx + 1]), x, &right);      \
    ts_vec3_add(&right, &(prev_value[idx + 1]), &right);       \
    ts_vec3_add(&left, &right, &(cur_first_der[idx]))

#define BEZIER_COMPUTE_NEXT_SECOND_DER                          \
    ts_vec3_scale(&(prev_second_der[idx]), one_minus_x, &left); \
    ts_vec3_sub(&left, &(prev_first_der[idx]), &left);          \
    ts_vec3_sub(&left, &(prev_first_der[idx]), &left);          \
    ts_vec3_scale(&(prev_second_der[idx + 1]), x, &right);      \
    ts_vec3_add(&right, &(prev_first_der[idx + 1]), &right);    \
    ts_vec3_add(&right, &(prev_first_der[idx + 1]), &right);    \
    ts_vec3_add(&left, &right, &(cur_second_der[idx]))

ts_vec3* ts_bezier(const ts_bezier_state* inst, ts_f64 x, ts_vec3* result) {
    ts_vec3* prev_value = BEZIER_SEGMENT(0);
    ts_vec3* cur_value = BEZIER_SEGMENT(1);

    ts_usize i;
    for (i = 0; i < inst->num_points; ++i) {
        vec3_copy(inst->points[i], prev_value[i]);
    }

    ts_f64 one_minus_x = 1.0 - x;
    ts_usize level;
    for (level = 1; level < inst->num_points; ++level) {
        ts_usize idx;
        for (idx = 0; idx < (inst->num_points - level); ++idx) {
            ts_vec3 left;
            ts_vec3 right;

            BEZIER_COMPUTE_NEXT_VALUE;
        }

        ts_ptr temp;
        BEZIER_SWAP_SEGMENT(cur_value, prev_value);
    }

    vec3_copy(*prev_value, *result);
    return result;
}

ts_vec3* ts_bezier_d(const ts_bezier_state* inst, ts_f64 x, ts_vec3* value, ts_vec3* result) {
    ts_vec3* prev_value = BEZIER_SEGMENT(0);
    ts_vec3* prev_first_der = BEZIER_SEGMENT(1);
    ts_vec3* cur_value = BEZIER_SEGMENT(2);
    ts_vec3* cur_first_der = BEZIER_SEGMENT(3);

    ts_usize i;
    for (i = 0; i < inst->num_points; ++i) {
        vec3_copy(inst->points[i], prev_value[i]);
        vec3_zero(prev_first_der[i]);
    }

    ts_f64 one_minus_x = 1.0 - x;
    ts_usize level;
    for (level = 1; level < inst->num_points; ++level) {
        ts_usize idx;
        for (idx = 0; idx < (inst->num_points - level); ++idx) {
            ts_vec3 left;
            ts_vec3 right;

            BEZIER_COMPUTE_NEXT_VALUE;
            BEZIER_COMPUTE_NEXT_FIRST_DER;
        }

        ts_ptr temp;
        BEZIER_SWAP_SEGMENT(cur_value, prev_value);
        BEZIER_SWAP_SEGMENT(cur_first_der, prev_first_der);
    }

    if (value) {
        vec3_copy(*prev_value, *value);
    }
    vec3_copy(*prev_first_der, *result);
    return result;
}

ts_vec3* ts_bezier_dd(const ts_bezier_state* inst, ts_f64 x, ts_vec3* value, ts_vec3* first_der, ts_vec3* result) {
    ts_vec3* prev_value = BEZIER_SEGMENT(0);
    ts_vec3* prev_first_der = BEZIER_SEGMENT(1);
    ts_vec3* prev_second_der = BEZIER_SEGMENT(2);
    ts_vec3* cur_value = BEZIER_SEGMENT(3);
    ts_vec3* cur_first_der = BEZIER_SEGMENT(4);
    ts_vec3* cur_second_der = BEZIER_SEGMENT(5);

    ts_usize i;
    for (i = 0; i < inst->num_points; ++i) {
        vec3_copy(inst->points[i], prev_value[i]);
        vec3_zero(prev_first_der[i]);
        vec3_zero(prev_second_der[i]);
    }

    ts_f64 one_minus_x = 1.0 - x;
    ts_usize level;
    for (level = 1; level < inst->num_points; ++level) {
        ts_usize idx;
        for (idx = 0; idx < (inst->num_points - level); ++idx) {
            ts_vec3 left;
            ts_vec3 right;

            BEZIER_COMPUTE_NEXT_VALUE;
            BEZIER_COMPUTE_NEXT_FIRST_DER;
            BEZIER_COMPUTE_NEXT_SECOND_DER;
        }

        ts_ptr temp;
        BEZIER_SWAP_SEGMENT(cur_value, prev_value);
        BEZIER_SWAP_SEGMENT(cur_first_der, prev_first_der);
        BEZIER_SWAP_SEGMENT(cur_second_der, prev_second_der);
    }

    if (value) {
        vec3_copy(*prev_value, *value);
    }
    if (first_der) {
        vec3_copy(*prev_first_der, *first_der);
    }
    vec3_copy(*prev_second_der, *result);
    return result;
}

ts_poly5_state* ts_poly5_init_constrained(const ts_vec3* start, const ts_vec3* start_d, const ts_vec3* start_dd,
                                          const ts_vec3* end, const ts_vec3* end_d, const ts_vec3* end_dd,
                                          ts_poly5_state* inst) {
    vec3_copy(*start, inst->coeffs[0]);
    vec3_copy(*start_d, inst->coeffs[1]);
    ts_vec3_scale(start_dd, 0.5, &(inst->coeffs[2]));

    ts_vec3 c, d, e, temp;
    ts_vec3_scale(start_dd, 0.5, &temp);
    ts_vec3_add(start, ts_vec3_add(start_d, &temp, &temp), &temp);
    ts_vec3_sub(end, &temp, &c);

    ts_vec3_add(start_d, start_dd, &temp);
    ts_vec3_sub(end_d, &temp, &d);

    ts_vec3_sub(end_dd, start_dd, &e);

    ts_vec3_scale(&c, 10, &(inst->coeffs[3]));
    ts_vec3_scale(&d, -4, &temp);
    ts_vec3_add(&(inst->coeffs[3]), &temp, &(inst->coeffs[3]));
    ts_vec3_scale(&e, 0.5, &temp);
    ts_vec3_add(&(inst->coeffs[3]), &temp, &(inst->coeffs[3]));

    ts_vec3_scale(&c, -15, &(inst->coeffs[4]));
    ts_vec3_scale(&d, 7, &temp);
    ts_vec3_add(&(inst->coeffs[4]), &temp, &(inst->coeffs[4]));
    ts_vec3_scale(&e, -1, &temp);
    ts_vec3_add(&(inst->coeffs[4]), &temp, &(inst->coeffs[4]));

    ts_vec3_scale(&c, 6, &(inst->coeffs[5]));
    ts_vec3_scale(&d, -3, &temp);
    ts_vec3_add(&(inst->coeffs[5]), &temp, &(inst->coeffs[5]));
    ts_vec3_scale(&e, 0.5, &temp);
    ts_vec3_add(&(inst->coeffs[5]), &temp, &(inst->coeffs[5]));

    return inst;
}

ts_vec3* ts_poly5(const ts_poly5_state* inst, ts_f64 x, ts_vec3* result) {
    vec3_copy(inst->coeffs[0], *result);

    ts_usize i;
    for (i = 1; i < 6; ++i) {
        ts_f64 factor = ts_pow(x, i);
        ts_vec3 temp;
        ts_vec3_add(ts_vec3_scale(&(inst->coeffs[i]), factor, &temp), result, result);
    }

    return result;
}

ts_vec3* ts_poly5_d(const ts_poly5_state* inst, ts_f64 x, ts_vec3* result) {
    vec3_copy(inst->coeffs[1], *result);

    ts_usize i;
    for (i = 2; i < 6; ++i) {
        ts_f64 factor = i * ts_pow(x, i - 1);
        ts_vec3 temp;
        ts_vec3_add(ts_vec3_scale(&(inst->coeffs[i]), factor, &temp), result, result);
    }

    return result;
}

ts_vec3* ts_poly5_dd(const ts_poly5_state* inst, ts_f64 x, ts_vec3* result) {
    ts_vec3_scale(&(inst->coeffs[2]), 2, result);

    ts_usize i;
    for (i = 3; i < 6; ++i) {
        ts_f64 factor = i * (i - 1) * ts_pow(x, i - 2);
        ts_vec3 temp;
        ts_vec3_add(ts_vec3_scale(&(inst->coeffs[i]), factor, &temp), result, result);
    }

    return result;
}
