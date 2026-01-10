#ifndef TEST_COMMON_H_
#define TEST_COMMON_H_

#include <assert.h>

#define EPS 1e-14
#define assert_eps(exp, act) assert((exp) - EPS < (act) && (exp) + EPS > (act))
#define assert_vec3(exp, act) \
    assert_eps(exp.x, act.x); \
    assert_eps(exp.y, act.y); \
    assert_eps(exp.z, act.z)
#define assert_quat(exp, act) assert_vec3((exp), (act)), assert_eps(exp.w, act.w)
#define assert_mat3x3(exp, act) \
    assert_eps(exp.xx, act.xx); \
    assert_eps(exp.xy, act.xy); \
    assert_eps(exp.xz, act.xz); \
    assert_eps(exp.yx, act.yx); \
    assert_eps(exp.yy, act.yy); \
    assert_eps(exp.yz, act.yz); \
    assert_eps(exp.zx, act.zx); \
    assert_eps(exp.zy, act.zy); \
    assert_eps(exp.zz, act.zz)

#endif  // TEST_COMMON_H_
