#include "common.h"
#include "ts.h"

void test_vec3_add(void) {
    ts_vec3 a = {1, 2, 3};
    ts_vec3 b = {4, 5, 6};
    ts_vec3 act = {0};
    ts_vec3 exp = {5, 7, 9};

    ts_vec3_add(&a, &b, &act);
    assert_vec3(exp, act);
}

void test_vec3_sub(void) {
    ts_vec3 a = {1, 2, 3};
    ts_vec3 b = {4, 5, 6};
    ts_vec3 act = {0};
    ts_vec3 exp = {-3, -3, -3};

    ts_vec3_sub(&a, &b, &act);
    assert_vec3(exp, act);
}

void test_vec3_scale(void) {
    ts_vec3 v = {1, 2, 3};
    ts_f64 factor = 2.5;
    ts_vec3 act = {0};
    ts_vec3 exp = {2.5, 5, 7.5};

    ts_vec3_scale(&v, factor, &act);
    assert_vec3(exp, act);
}

void test_vec3_dot(void) {
    ts_vec3 a = {1, 2, 3};
    ts_vec3 b = {4, 5, 6};
    ts_f64 exp = 32.0;
    
    ts_f64 act = ts_vec3_dot(&a, &b);
    assert_eps(exp, act);
}

void test_vec3_cross(void) {
    ts_vec3 a = {1, 2, 3};
    ts_vec3 b = {4, 5, 6};
    ts_vec3 act = {0};
    ts_vec3 exp = {-3, 6, -3};
    
    ts_vec3_cross(&a, &b, &act);
    assert_vec3(exp, act);
}

void test_vec3_norm_l2(void) {
    ts_vec3 v = {1, 2, 3};
    ts_f64 exp = ts_sqrt(14);

    ts_f64 act = ts_vec3_norm_l2(&v);
    assert_eps(exp, act);
}

void test_vec3_normalize(void) {
    ts_vec3 v = {1, 2, 3};
    ts_vec3 act = {0};
    ts_vec3 exp = {1.0/ts_sqrt(14), 2 / ts_sqrt(14), 3 / ts_sqrt(14)};

    ts_vec3_normalize(&v, &act);
    assert_vec3(exp, act);
}

int main(void) {
    test_vec3_add();
    test_vec3_sub();
    test_vec3_scale();
    test_vec3_dot();
    test_vec3_cross();
    test_vec3_norm_l2();
    test_vec3_normalize();
    return 0;
}
