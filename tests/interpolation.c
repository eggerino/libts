#include "common.h"
#include "ts.h"

void test_linear3(void) {
    ts_linear3_state s = {0};
    ts_vec3 start = {1, 2, 3};
    ts_vec3 end = {4, 6, 8};
    ts_f64 x = 0.25;
    ts_vec3 act = {0};
    ts_vec3 exp = {1.75, 3, 4.25};

    ts_linear3_init(&start, &end, &s);
    ts_linear3(&s, x, &act);
    assert_vec3(exp, act);
}

void test_linear3_d(void) {
    ts_linear3_state s = {0};
    ts_vec3 start = {1, 2, 3};
    ts_vec3 end = {4, 6, 8};
    ts_vec3 act = {0};
    ts_vec3 exp = {3, 4, 5};

    ts_linear3_init(&start, &end, &s);
    ts_linear3_d(&s, &act);
    assert_vec3(exp, act);
}

void test_linear3_dd(void) {
    ts_vec3 act = {0};
    ts_vec3 exp = {0, 0, 0};

    ts_linear3_dd(&act);
    assert_vec3(exp, act);
}

// TODO slerp, bezier, poly5

int main(void) {
    test_linear3();
    test_linear3_d();
    test_linear3_dd();
    return 0;
}
