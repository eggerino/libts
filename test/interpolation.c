#include "common.h"
#include "ts.h"

void test_interpolate_linear1(void) {
    assert_eps(0, ts_interpolate_linear1(0, 1, 0));
    assert_eps(1, ts_interpolate_linear1(0, 1, 1));
    assert_eps(2, ts_interpolate_linear1(0, 1, 2));
    assert_eps(-1, ts_interpolate_linear1(0, 1, -1));
    assert_eps(1, ts_interpolate_linear1(1, 1, -1));
}

void test_interpolate_linear3(void) {
    ts_vec3 expected = {1.75, 2, 2.5};
    ts_vec3 actual = {0};
    ts_f64 x = 0.25;

    ts_interpolate_linear3(&((ts_vec3){1, 2, 3}), &((ts_vec3){4, 2, 1}), x, &actual);

    assert_vec3(expected, actual);
}

void test_interpolate_slerp(void) {
    ts_quat expected = {0.8660254037844386, 0.28867513459481287, 0.28867513459481287, 0.28867513459481287};
    ts_quat actual = {0};
    ts_f64 x = 0.5;

    ts_interpolate_slerp(&((ts_quat){1, 0, 0, 0}), &((ts_quat){.5, .5, .5, .5}), x, &actual);

    assert_quat(expected, actual);
}

int main(void) {
    test_interpolate_linear1();
    test_interpolate_linear3();
    test_interpolate_slerp();
    return 0;
}
