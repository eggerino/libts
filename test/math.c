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

void test_quat_add(void) {
    ts_quat a = {1, 2, 3, 4};
    ts_quat b = {4, 5, 6, 7};
    ts_quat act = {0};
    ts_quat exp = {5, 7, 9, 11};

    ts_quat_add(&a, &b, &act);
    assert_quat(exp, act);
}

void test_quat_sub(void) {
    ts_quat a = {1, 2, 3, 4};
    ts_quat b = {4, 5, 6, 7};
    ts_quat act = {0};
    ts_quat exp = {-3, -3, -3, -3};

    ts_quat_sub(&a, &b, &act);
    assert_quat(exp, act);
}

void test_quat_scale(void) {
    ts_quat q = {1, 2, 3, 4};
    ts_f64 factor = 2.5;
    ts_quat act = {0};
    ts_quat exp = {2.5, 5, 7.5, 10};

    ts_quat_scale(&q, factor, &act);
    assert_quat(exp, act);
}

void test_quat_dot(void) {
    ts_quat a = {1, 2, 3, 4};
    ts_quat b = {4, 5, 6, 7};
    ts_f64 exp = 60;
    
    ts_f64 act = ts_quat_dot(&a, &b);
    assert_eps(exp, act);
}

void test_quat_mul(void) {
    ts_quat a = {1, 2, 3, 4};
    ts_quat b = {4, 5, 6, 7};
    ts_quat act = {0};
    ts_quat exp = {-52, 10, 24, 20};

    ts_quat_mul(&a, &b, &act);
    assert_quat(exp, act);
}

void test_quat_normalize(void) {
    ts_quat q = {1, 2, 3, 4};
    ts_quat act = {0};
    ts_quat exp = {1.0 / ts_sqrt(30), 2.0 / ts_sqrt(30), 3.0 / ts_sqrt(30), 4.0 / ts_sqrt(30)};

    ts_quat_normalize(&q, &act);
    assert_quat(exp, act);
}

int main(void) {
    test_vec3_add();
    test_vec3_sub();
    test_vec3_scale();
    test_vec3_dot();
    test_vec3_cross();
    test_vec3_norm_l2();
    test_vec3_normalize();

    test_quat_add();
    test_quat_sub();
    test_quat_scale();
    test_quat_dot();
    test_quat_mul();
    test_quat_normalize();
    return 0;
}
