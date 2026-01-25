/*
 * Library for trajectory sampling centered around robotics.
 * TODO: Describe library
 */

#ifndef TS_H_
#define TS_H_

#include "ts_sys.h"

/* Precision of the library. Values smaller than this are considered zero. */
#define TS_EPS 1e-15

/* Result of a function which needs dynamic memory */
typedef enum {
    TS_ALLOC_OK = 0, /* No error */
    TS_ALLOC_ERR,    /* Allocation failed */
} ts_allocation_error;

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
    ts_f64 w; /* W-component (real part) */
    ts_f64 x; /* X-component (first imaginary part) */
    ts_f64 y; /* Y-component (second imaginary part) */
    ts_f64 z; /* Z-component (third imaginary part) */
} ts_quat;

/* 3x3 matrix */
typedef struct {
    ts_f64 xx; /* First row and first column */
    ts_f64 xy; /* First row and second column */
    ts_f64 xz; /* First row and third column */
    ts_f64 yx; /* Second row and first column */
    ts_f64 yy; /* Second row and second column */
    ts_f64 yz; /* Second row and third column */
    ts_f64 zx; /* Third row and first column */
    ts_f64 zy; /* Third row and second column */
    ts_f64 zz; /* Third row and third column */
} ts_mat3x3;

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

/* Adds the matrices a and b. */
ts_mat3x3* ts_mat3x3_add(const ts_mat3x3* a, const ts_mat3x3* b, ts_mat3x3* result);

/* Subtracts the matrix b from the matrix a. */
ts_mat3x3* ts_mat3x3_sub(const ts_mat3x3* a, const ts_mat3x3* b, ts_mat3x3* result);

/* Multiplies the matrices a and b. */
ts_mat3x3* ts_mat3x3_mul(const ts_mat3x3* a, const ts_mat3x3* b, ts_mat3x3* result);

/* Multiplies the matrix a with the vector b. */
ts_vec3* ts_mat3x3_vec3_mul(const ts_mat3x3* a, const ts_vec3* b, ts_vec3* result);

/* Converts the matrix m to a unit quaternion. It is expected that m represents a valid orientation. */
ts_quat* ts_mat3x3_to_quat(const ts_mat3x3* m, ts_quat* result);

/* Converts the quaternion q to an orientation matrix. It is expected that q is a unit quaternion. */
ts_mat3x3* ts_quat_to_mat3x3(const ts_quat* q, ts_mat3x3* result);

/* Converts the vector v to an imaginary quatenion. */
ts_quat* ts_vec3_to_imag_quat(const ts_vec3* v, ts_quat* result);

/* Converts the imaginary part of the quaternion q to a vector. */
ts_vec3* ts_quat_imag_to_vec3(const ts_quat* q, ts_vec3* result);

/* Computes the axis angle representation of the quaternion as a scaled vector. */
ts_vec3* ts_quat_to_axis_angle_vec3(const ts_quat* q, ts_vec3* result);

/* Compute the unit quaternion representation of the scaled axis angle vector. */
ts_quat* ts_vec3_axis_angle_to_quat(const ts_vec3* v, ts_quat* result);

/*
 * +----------------------+
 * | Interpolation module |
 * +----------------------+
 *
 * Data types and operations for general interpolation.
 *
 * If these functions return a struct, the result memory is passed as a pointer.
 * The same pointer is also returned, so you can build expresseions like:
 *
 * y = ts_linear1_at(ts_linear1_init(start, end, &l1), x);
 *
 * It is always safe to pass the same pointer as the result parameter to reassign the result.
 */

/*
 * Linear interpolation in three dimensional space.
 */

/* State of a three dimensional linear interpolation */
typedef struct {
    ts_vec3 start; /* Start value */
    ts_vec3 delta; /* Difference between start and end value */
} ts_linear3_state;

/* Initialize a three dimensional linear interpolation state. */
ts_linear3_state* ts_linear3_init(const ts_vec3* start, const ts_vec3* end, ts_linear3_state* inst);

/* Interpolate at the value x. */
ts_vec3* ts_linear3(const ts_linear3_state* inst, ts_f64 x, ts_vec3* result);

/* Compute the first derivative with respect to x. */
ts_vec3* ts_linear3_d(const ts_linear3_state* inst, ts_vec3* result);

/* Compute the second derivative with respect to x. */
ts_vec3* ts_linear3_dd(ts_vec3* result);

/*
 * Spherical linear interpolation of quaternions.
 */

/* State of a spherical linear interpolation */
typedef struct {
    ts_quat start;             /* Start value */
    ts_quat end;               /* End value*/
    ts_f64 omega;              /* Angle between start and end value */
    ts_f64 one_over_sin_omega; /* Precomputed value */
} ts_slerp_state;

/* Initialize a spherical linear interpolation state. */
ts_slerp_state* ts_slerp_init(const ts_quat* start, const ts_quat* end, ts_slerp_state* inst);

/* Interpolate at the value x. */
ts_quat* ts_slerp(const ts_slerp_state* inst, ts_f64 x, ts_quat* result);

/* Compute the first derivative with respect to x at x. */
ts_quat* ts_slerp_d(const ts_slerp_state* inst, ts_f64 x, ts_quat* result);

/* Compute the second derivative with respect to x at x. */
ts_quat* ts_slerp_dd(const ts_slerp_state* inst, ts_f64 x, ts_quat* result);

/*
 * Bezier interpolation in three dimensional space.
 */

/* State of a three dimensional bezier interpolation. It holds data to dynamic memory. */
typedef struct {
    ts_vec3* points;     /* Array of the control points */
    ts_usize num_points; /* Number of control points in the array */
    ts_ptr buffer;       /* Memory for computation */
} ts_bezier_state;

/* Initialize a three dimensional bezier interpolation state. This function uses dynamic memory. Make sure to call
 * `ts_bezier_deinit` after usage. */
ts_allocation_error ts_bezier_init(const ts_vec3* points, ts_usize num_points, ts_bezier_state* inst);

/* Deinitialize a three dimensional bezier interpolation state. */
ts_none ts_bezier_deinit(ts_bezier_state* inst);

/* Interpolate at the value x */
ts_vec3* ts_bezier(const ts_bezier_state* inst, ts_f64 x, ts_vec3* result);

/* Compute up to the first derivative with respect to x at x. It returns the first derivative pointer. The value pointer can
 * be `NULL` if only the derivative is needed. */
ts_vec3* ts_bezier_d(const ts_bezier_state* inst, ts_f64 x, ts_vec3* value, ts_vec3* result);

/* Compute up to the second derivative with respect to x at x. It returns the second derivative pointer. The value and first
 * derivative pointer can be `NULL` if they are not needed. */
ts_vec3* ts_bezier_dd(const ts_bezier_state* inst, ts_f64 x, ts_vec3* value, ts_vec3* first_der, ts_vec3* result);

/*
 * Polynomial interpolation of order 5 in three dimensional space.
 */

/* State of a three dimensional polynomial of order 5 */
typedef struct {
    ts_vec3 coeffs[6]; /* Coefficients of the polynomial */
} ts_poly5_state;

/* Initialize a three dimensional polynomial of order 5 so that the start and end condition in terms of value, first and
 * second derivative are met. */
ts_poly5_state* ts_poly5_init_constrained(const ts_vec3* start, const ts_vec3* start_d, const ts_vec3* start_dd,
                                          const ts_vec3* end, const ts_vec3* end_d, const ts_vec3* end_dd,
                                          ts_poly5_state* inst);

/* Interpolate at the value x. */
ts_vec3* ts_poly5(const ts_poly5_state* inst, ts_f64 x, ts_vec3* result);

/* Compute the first derivative with respect to x at x. */
ts_vec3* ts_poly5_d(const ts_poly5_state* inst, ts_f64 x, ts_vec3* result);

/* Compute the second derivative with respect to x at x. */
ts_vec3* ts_poly5_dd(const ts_poly5_state* inst, ts_f64 x, ts_vec3* result);

typedef struct {
    ts_vec3 pos;
    ts_quat orient;
} ts_pose;

typedef struct {
    ts_pose pose;
    ts_pose velocity;
    ts_pose acceleration;
} ts_pose_state;

#endif  // TS_H_