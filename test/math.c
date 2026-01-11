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
    ts_vec3 exp = {1.0 / ts_sqrt(14), 2 / ts_sqrt(14), 3 / ts_sqrt(14)};

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

void test_mat3x3_add(void) {
    ts_mat3x3 a = {
        1, 2, 3, 4, 5, 6, 7, 8, 9,
    };
    ts_mat3x3 b = {
        4, 5, 6, 7, 8, 9, 1, 2, 3,
    };
    ts_mat3x3 act = {0};
    ts_mat3x3 exp = {
        5, 7, 9, 11, 13, 15, 8, 10, 12,
    };

    ts_mat3x3_add(&a, &b, &act);
    assert_mat3x3(exp, act);
}

void test_mat3x3_sub(void) {
    ts_mat3x3 a = {
        1, 2, 3, 4, 5, 6, 7, 8, 9,
    };
    ts_mat3x3 b = {
        4, 5, 6, 7, 8, 9, 1, 2, 3,
    };
    ts_mat3x3 act = {0};
    ts_mat3x3 exp = {
        -3, -3, -3, -3, -3, -3, 6, 6, 6,
    };

    ts_mat3x3_sub(&a, &b, &act);
    assert_mat3x3(exp, act);
}

void test_mat3x3_mul(void) {
    ts_mat3x3 a = {
        1, 2, 3, 4, 5, 6, 7, 8, 9,
    };
    ts_mat3x3 b = {
        4, 5, 6, 7, 8, 9, 1, 2, 3,
    };
    ts_mat3x3 act = {0};
    ts_mat3x3 exp = {
        21, 27, 33, 57, 72, 87, 93, 117, 141,
    };

    ts_mat3x3_mul(&a, &b, &act);
    assert_mat3x3(exp, act);
}

void test_mat3x3_vec3_mul(void) {
    ts_mat3x3 a = {
        1, 2, 3, 4, 5, 6, 7, 8, 9,
    };
    ts_vec3 b = {4, 5, 6};
    ts_vec3 act = {0};
    ts_vec3 exp = {32, 77, 122};

    ts_mat3x3_vec3_mul(&a, &b, &act);
    assert_vec3(exp, act);
}

void test_mat3x3_to_quat(void) {
    ts_mat3x3 m = {
        0.41198225991309989, 0.05872672981356003, 0.90929742922092005,
        -0.6812427255054001, -0.64287283953050012, 0.35017557591120008,
        0.60512725316939997, -0.76371837494208006, -0.22484507008615995,
    };
    ts_quat act = {0};
    ts_quat exp = {-0.36887138766997318, 0.75493381109501456, -0.20614919310918339, 0.50150908323637811};

    ts_mat3x3_to_quat(&m, &act);
    assert_quat(exp, act);
}

void test_quat_to_mat3x3(void) {
    ts_quat q = {-0.3688714, 0.7549338, -0.2061492, 0.5015091};
    ts_mat3x3 act = {0};
    ts_mat3x3 exp = {
        0.41198225991309989, 0.05872672981356003, 0.90929742922092005,
        -0.6812427255054001, -0.64287283953050012, 0.35017557591120008,
        0.60512725316939997, -0.76371837494208006, -0.22484507008615995,
    };

    ts_quat_to_mat3x3(&q, &act);
    assert_mat3x3(exp, act);
}

void test_vec3_to_imag_quat(void) {
    ts_vec3 v = {2, 3, 4};
    ts_quat act = {0};
    ts_quat exp = {0, 2, 3, 4};

    ts_vec3_to_imag_quat(&v, &act);
    assert_quat(exp, act);
}

void test_quat_imag_to_vec3(void) {
    ts_quat q = {1, 2, 3, 4};
    ts_vec3 act = {0};
    ts_vec3 exp = {2, 3, 4};

    ts_quat_imag_to_vec3(&q, &act);
    assert_vec3(exp, act);
}

// TODO Axis angle tests

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

    test_mat3x3_add();
    test_mat3x3_sub();
    test_mat3x3_mul();
    test_mat3x3_vec3_mul();

    test_mat3x3_to_quat();
    test_quat_to_mat3x3();
    test_vec3_to_imag_quat();
    test_quat_imag_to_vec3();
    return 0;
}
