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
    double x;
    double y;
    double z;
    double w;
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

// Interpolation functions
ts_f64 ts_interpolate_linear1(ts_f64 start, ts_f64 end, ts_f64 x);
ts_none ts_interpolate_linear3(const ts_vec3* start, const ts_vec3* end, ts_f64 x, ts_vec3* result);

#endif  // TS_H_