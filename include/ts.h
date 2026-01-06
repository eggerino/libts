/*
 * Library for trajectory sampling centered around robotics.
 * TODO: Describe library
 */

#ifndef TS_H_
#define TS_H_

#include "ts_sys.h"

// Common complex data types
typedef struct {
    double x;
    double y;
    double z;
} ts_vec3;

typedef struct {
    double w;
    double x;
    double y;
    double z;
} ts_quat;

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
ts_vec3* ts_vec3_add(const ts_vec3* a, const ts_vec3* b, ts_vec3* result);
ts_vec3* ts_vec3_sub(const ts_vec3* a, const ts_vec3* b, ts_vec3* result);
ts_f64 ts_quat_dot(const ts_quat* a, const ts_quat* b);

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