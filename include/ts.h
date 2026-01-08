/*
 * Library for trajectory sampling centered around robotics.
 * TODO: Describe library
 */

#ifndef TS_H_
#define TS_H_

#include "ts_sys.h"

/* Precision of the library. Values smaller than this are considered zero. */
#define TS_EPS 1e-15

/*
 * +-------------+
 * | Math module |
 * +-------------+
 *
 * Data types and operations for general purpose math.
 *
 * If these functions return a struct, the result memory is passed as a pointer.
 * The same pointer is also returned, so you can build expresseions like:
 *
 * ts_vec3_add(ts_vec3_scale(a, f1, a), ts_vec3_scale(b, f2, b), c);
 *
 * Which will compute c = a * f1 + b * f2
 *
 * It is always safe to pass the same pointer as the result parameter to reassign the result.
 */

/* Vector in 3-dimensional space */
typedef struct {
    ts_f64 x; /* X-component */
    ts_f64 y; /* Y-component */
    ts_f64 z; /* Z-component */
} ts_vec3;

/* Quaternion */
typedef struct {
    double w; /* W-component (real part) */
    double x; /* X-component (first imaginary part) */
    double y; /* Y-component (second imaginary part) */
    double z; /* Z-component (third imaginary part) */
} ts_quat;

/* Adds the vectors a and b. */
ts_vec3* ts_vec3_add(const ts_vec3* a, const ts_vec3* b, ts_vec3* result);

/* Subtracts the vector b from the vector a. */
ts_vec3* ts_vec3_sub(const ts_vec3* a, const ts_vec3* b, ts_vec3* result);

/* Scales the vector v with the given factor. */
ts_vec3* ts_vec3_scale(const ts_vec3* v, ts_f64 factor, ts_vec3* result);

/* Computes the dot product of the vectors a and b. */
ts_f64 ts_vec3_dot(const ts_vec3* a, const ts_vec3* b);

/* Computes the cross product of the vectors a and b. */
ts_vec3* ts_vec3_cross(const ts_vec3* a, const ts_vec3* b, ts_vec3* result);

/*
 * Computes the L2 norm of the vector v.
 * It computes a square root. If you only need the square of this, consider using ts_vec3_dot instead.
 */
ts_f64 ts_vec3_norm_l2(const ts_vec3* v);

/* Scales the vector v so its scaled l2 norm equals one. If the l2 norm is near zero, the unit vector along the x-axis is
 * returned. So this function will always return a vector with the l2 norm of one. */
ts_vec3* ts_vec3_normalize(const ts_vec3* v, ts_vec3* result);

/* Adds the quaternions a and b. */
ts_quat* ts_quat_add(const ts_quat* a, const ts_quat* b, ts_quat* result);

/* Subtracts the quaternion b from the quaternion a. */
ts_quat* ts_quat_sub(const ts_quat* a, const ts_quat* b, ts_quat* result);

/* Scales the quaternion q with the given factor. */
ts_quat* ts_quat_scale(const ts_quat* q, ts_f64 factor, ts_quat* result);

/* Computes the dot product of the quaternions a and b as if they where 4-dimensional vectors. */
ts_f64 ts_quat_dot(const ts_quat* a, const ts_quat* b);

/* Multiplies the quaternions a and b. */
ts_quat* ts_quat_mul(const ts_quat* a, const ts_quat* b, ts_quat* result);

/* Scales the quaternion q so its scaled l2 norm equals one. If the l2 norm is near zero, the unit quaternion along the
 * w-axis is returned. So this function will always return a quaternion  with the l2 norm of one.*/
ts_quat* ts_quat_normalize(const ts_quat* q, ts_quat* result);






typedef struct {
    ts_vec3 pos;
    ts_quat orient;
} ts_pose;

typedef struct {
    ts_pose pose;
    ts_pose velocity;
    ts_pose acceleration;
} ts_pose_state;

// Common operations

// Interpolation functions

ts_f64 ts_interpolate_linear1(ts_f64 start, ts_f64 end, ts_f64 x);
ts_f64 ts_interpolate_linear1_d(ts_f64 start, ts_f64 end);
ts_f64 ts_interpolate_linear1_dd();
ts_vec3* ts_interpolate_linear3(const ts_vec3* start, const ts_vec3* end, ts_f64 x, ts_vec3* result);
ts_vec3* ts_interpolate_linear3_d(const ts_vec3* start, const ts_vec3* end, ts_vec3* result);
ts_vec3* ts_interpolate_linear3_dd(ts_vec3* result);
ts_quat* ts_interpolate_slerp(const ts_quat* start, const ts_quat* end, ts_f64 x, ts_quat* result);
ts_quat* ts_interpolate_slerp_d(const ts_quat* start, const ts_quat* end, ts_f64 x, ts_f64 dx, ts_quat* result);
ts_quat* ts_interpolate_slerp_dd(const ts_quat* start, const ts_quat* end, ts_f64 x, ts_f64 dx, ts_f64 ddx, ts_quat* result);

#endif  // TS_H_