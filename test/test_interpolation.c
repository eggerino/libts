#include <assert.h>

#include "ts.h"

#define EPS 1e-15
#define assert_eps(exp, act) assert((exp) - EPS < (act) && (exp) + EPS > (act))

void test_interpolate_linear1(void) {
    assert_eps(0, ts_interpolate_linear1(0, 1, 0));
    assert_eps(1, ts_interpolate_linear1(0, 1, 1));
    assert_eps(2, ts_interpolate_linear1(0, 1, 2));
    assert_eps(-1, ts_interpolate_linear1(0, 1, -1));
    assert_eps(1, ts_interpolate_linear1(1, 1, -1));
}

void test_interpolate_linear3(void) {
    ts_vec3 a = {1, 2, 3};
    ts_vec3 b = {4, 2, 1};
    ts_vec3 result = {0};

    ts_interpolate_linear3(&a, &b, 0.25, &result);

    assert_eps(1.75, result.x);
    assert_eps(2, result.y);
    assert_eps(2.5, result.z);
}

int main(void) {
    test_interpolate_linear1();
    test_interpolate_linear3();
}
